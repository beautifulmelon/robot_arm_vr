#!/usr/bin/env python3
"""방향 뷰어 — 브라우저에서 yaw/미러를 돌려보며 눈으로 확인한다.

    python3 serve_viewer.py                 # http://localhost:8770
    python3 serve_viewer.py --port 9000
    python3 serve_viewer.py --urdf 자기URDF.urdf

무엇을 하나
-----------
  · 관절 슬라이더를 움직이면 로봇이 움직인다
  · **yaw / 미러 버튼**으로 8조합을 바로 눈으로 비교한다
  · 화면 그리는 코드는 저희 실물 텔레옵이 쓰는 `robot_view.js` **그대로**다.
    즉 여기서 보이는 것이 VR 헤드셋 안에서 보이는 것과 같은 규약이다.

의존성
------
  **numpy 뿐이다.** 기구학(FK)을 이 파일 안에 순수 numpy 로 넣었다.
  placo·pinocchio·ROS 없이 돈다. STL 메시도 필요 없다 —
  링크 바운딩 박스는 geometry_*.json 에 미리 계산해 두었다.

  (이 FK 는 pinocchio 와 대조 검증했다. 최대 오차 1e-15 m / 1e-15 rad)

자기 URDF 로 볼 때
------------------
  `--urdf` 로 주면 박스 크기를 모르므로 각 링크를 **작은 정육면체**로 그린다.
  관절 위치와 회전 방향을 보는 데는 충분하다. 형상까지 보려면
  geometry_*.json 과 같은 형식으로 만들어 `--geometry` 로 주면 된다.
"""

from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import numpy as np

HERE = Path(__file__).resolve().parent
PKG = HERE.parent


# ════════════════════════════════════════════════════════════════════════════
#  좌표 매핑 — transforms.py 와 같은 값이어야 한다
# ════════════════════════════════════════════════════════════════════════════
R_WEBXR_TO_ROBOT = np.array([[0.0, 0.0, -1.0],
                             [-1.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0]])
MIRROR_Y = np.diag([1.0, -1.0, 1.0])


def frame_mapping(yaw_deg: float, mirror: bool) -> np.ndarray:
    a = np.radians(float(yaw_deg))
    c, s = np.cos(a), np.sin(a)
    yaw = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    R = yaw @ R_WEBXR_TO_ROBOT
    return MIRROR_Y @ R if mirror else R


# 사람 손 기준 방향 (WebXR RUB)
HAND_DIRS = [("손 오른쪽", [1.0, 0, 0]), ("손 앞", [0, 0, -1.0]), ("손 위", [0, 1.0, 0])]
_ROBOT_DIR_NAMES = {
    (1, 0, 0): "앞 (+x)", (-1, 0, 0): "뒤 (−x)",
    (0, 1, 0): "왼쪽 (+y)", (0, -1, 0): "오른쪽 (−y)",
    (0, 0, 1): "위 (+z)", (0, 0, -1): "아래 (−z)",
}


def hand_dir_table(yaw_deg: float, mirror: bool) -> list[dict]:
    """"손을 이쪽으로 밀면 로봇 EE 가 저쪽으로" 표.

    ★ 이 계산을 브라우저에 또 두지 않는다. 두 벌이 생기면 언젠가 어긋나고,
      그때는 "방향이 안 맞는다" 로 보이지만 원인은 두 구현의 불일치다.
      (가이드 §6-2 에 적은 것과 같은 이유다)
    """
    M = frame_mapping(yaw_deg, mirror)
    out = []
    for label, v in HAND_DIRS:
        r = M @ np.array(v)
        key = tuple(int(round(x)) for x in r)
        out.append({"from": label,
                    "to": _ROBOT_DIR_NAMES.get(key, ", ".join(f"{x:.2f}" for x in r))})
    return out


# ════════════════════════════════════════════════════════════════════════════
#  URDF 기구학 — 순수 numpy
# ════════════════════════════════════════════════════════════════════════════
def rpy_to_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """로드리게스."""
    n = float(np.linalg.norm(axis))
    if n < 1e-12 or abs(angle) < 1e-15:
        return np.eye(3)
    k = axis / n
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


class UrdfChain:
    """URDF 를 읽어 링크 world pose 를 계산한다. 트리 구조를 지원한다."""

    def __init__(self, urdf_path: str | Path):
        self.path = Path(urdf_path)
        root = ET.parse(self.path).getroot()

        self.joints = []          # 선언 순서 (= 지령 배열 순서)
        self.children = {}        # parent link -> [joint dict]
        all_children = set()
        all_parents = set()

        for j in root.findall("joint"):
            og = j.find("origin")
            ax = j.find("axis")
            lim = j.find("limit")
            xyz = [float(v) for v in (og.get("xyz", "0 0 0").split() if og is not None
                                      else ["0", "0", "0"])]
            rpy = [float(v) for v in (og.get("rpy", "0 0 0").split() if og is not None
                                      else ["0", "0", "0"])]
            info = {
                "name": j.get("name"),
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "t": np.array(xyz, dtype=float),
                "R": rpy_to_matrix(*rpy),
                "axis": np.array([float(v) for v in ax.get("xyz", "1 0 0").split()],
                                 dtype=float) if ax is not None else np.array([1.0, 0, 0]),
                "lower": float(lim.get("lower")) if lim is not None and lim.get("lower")
                else -np.pi,
                "upper": float(lim.get("upper")) if lim is not None and lim.get("upper")
                else np.pi,
            }
            self.children.setdefault(info["parent"], []).append(info)
            all_children.add(info["child"])
            all_parents.add(info["parent"])
            if info["type"] in ("revolute", "continuous"):
                self.joints.append(info)

        roots = all_parents - all_children
        self.base = sorted(roots)[0] if roots else "base_link"

        # 베이스부터 너비우선으로 훑어 링크 순서와 깊이를 구한다.
        self.link_names, self.depth = [self.base], {self.base: 0}
        queue = [self.base]
        while queue:
            parent = queue.pop(0)
            for j in self.children.get(parent, []):
                c = j["child"]
                if c in self.depth:
                    continue
                self.depth[c] = self.depth[parent] + 1
                self.link_names.append(c)
                queue.append(c)

        # EE 링크 — 이름으로 먼저 찾고, 없으면 가장 깊은 링크.
        # ★ "선언 순서상 마지막" 으로 고르면 안 된다. camera_mount 처럼 팔 중간에
        #   달린 부속이 뒤에 선언돼 있으면 그게 EE 로 잡힌다.
        hint = [n for n in self.link_names
                if any(k in n.lower() for k in ("hand", "tool", "gripper", "_ee", "tcp"))]
        self.ee_link = hint[-1] if hint else max(self.link_names, key=lambda n: self.depth[n])

    @property
    def joint_names(self) -> list[str]:
        return [j["name"] for j in self.joints]

    @property
    def dof(self) -> int:
        return len(self.joints)

    def fk(self, q) -> dict[str, np.ndarray]:
        """관절각 → 링크별 4x4 world pose."""
        q = np.asarray(q, dtype=float).ravel()
        qi = {j["name"]: (q[i] if i < len(q) else 0.0) for i, j in enumerate(self.joints)}
        out = {self.base: np.eye(4)}
        stack = [self.base]
        while stack:
            parent = stack.pop()
            Tp = out[parent]
            for j in self.children.get(parent, []):
                T = np.eye(4)
                T[:3, :3] = j["R"]
                T[:3, 3] = j["t"]
                if j["type"] in ("revolute", "continuous"):
                    Rj = np.eye(4)
                    Rj[:3, :3] = axis_rotation(j["axis"], qi.get(j["name"], 0.0))
                    T = T @ Rj
                out[j["child"]] = Tp @ T
                stack.append(j["child"])
        return out


def rotation_to_quat(R: np.ndarray) -> list[float]:
    """회전행렬 → xyzw 쿼터니언 (three.js 순서)."""
    tr = float(np.trace(R))
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s
            x, y, z = 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s
            x, y, z = (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s
            x, y, z = (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return [round(float(v), 6) for v in (x, y, z, w)]


# ════════════════════════════════════════════════════════════════════════════
#  서버
# ════════════════════════════════════════════════════════════════════════════
class Viewer:
    def __init__(self, urdf: Path, geometry: Path | None):
        self.chain = UrdfChain(urdf)
        self.name = urdf.name
        if geometry and geometry.exists():
            self.geometry = json.loads(geometry.read_text())
            known = {g["name"] for g in self.geometry}
            missing = [n for n in self.chain.link_names if n not in known]
            for n in missing:      # 형상을 모르는 링크는 작은 정육면체로
                self.geometry.append({"name": n, "center": [0, 0, 0],
                                      "size": [0.04, 0.04, 0.04], "joint": None})
        else:
            jl = {j["child"]: j["name"] for lst in self.chain.children.values() for j in lst
                  if j["type"] in ("revolute", "continuous")}
            self.geometry = [{"name": n, "center": [0, 0, 0], "size": [0.05, 0.05, 0.05],
                              "joint": jl.get(n)} for n in self.chain.link_names]

    def state(self, q, yaw: float, mirror: bool) -> dict:
        poses = self.chain.fk(q)
        M = np.eye(4)
        M[:3, :3] = np.linalg.inv(frame_mapping(yaw, mirror))   # ★ 뷰 = 제어 매핑의 역행렬
        ee = poses[self.chain.ee_link][:3, 3]
        joints = []
        for i, j in enumerate(self.chain.joints):
            v = float(q[i]) if i < len(q) else 0.0
            span = j["upper"] - j["lower"]
            near = span * 0.05
            st = "limit" if (v <= j["lower"] + 1e-6 or v >= j["upper"] - 1e-6) else \
                 ("near" if (v - j["lower"] < near or j["upper"] - v < near) else "ok")
            joints.append({"name": j["name"], "status": st,
                           "deg": round(np.degrees(v), 1)})
        return {
            "view_matrix": [round(float(v), 6) for v in M.reshape(-1)],   # row-major
            "geometry": self.geometry,
            "links": [{"name": n, "p": [round(float(v), 5) for v in T[:3, 3]],
                       "q": rotation_to_quat(T[:3, :3])} for n, T in poses.items()],
            "joints": joints,
            "ee_point": [round(float(v), 5) for v in ee],
            "robot": {"name": self.name, "dof": self.chain.dof, "ee": self.chain.ee_link},
            "mapping": {"yaw_deg": yaw, "mirror": mirror},
            "hand_dirs": hand_dir_table(yaw, mirror),
        }


STATIC = {
    "/": ("viewer.html", "text/html; charset=utf-8"),
    "/viewer.html": ("viewer.html", "text/html; charset=utf-8"),
    "/robot_view.js": ("../code/robot_view.js", "application/javascript"),
}


def make_handler(viewer: Viewer):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):            # 요청마다 찍으면 시끄럽다
            pass

        def _send(self, code, body: bytes, ctype: str):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)

            if u.path == "/state":
                p = parse_qs(u.query)
                q = [float(v) for v in p.get("q", [""])[0].split(",") if v.strip()]
                yaw = float(p.get("yaw", ["0"])[0])
                mirror = p.get("mirror", ["0"])[0] in ("1", "true", "True")
                body = json.dumps(viewer.state(q, yaw, mirror),
                                  allow_nan=False).encode()
                return self._send(200, body, "application/json")

            if u.path == "/info":
                body = json.dumps({
                    "robot": viewer.name,
                    "joints": [{"name": j["name"],
                                "lower": round(np.degrees(j["lower"]), 1),
                                "upper": round(np.degrees(j["upper"]), 1),
                                "axis": [float(v) for v in j["axis"]]}
                               for j in viewer.chain.joints],
                }).encode()
                return self._send(200, body, "application/json")

            if u.path.startswith("/vendor/"):
                f = HERE / "vendor" / Path(u.path).name
                if f.exists():
                    return self._send(200, f.read_bytes(), "application/javascript")
                return self._send(404, b"no vendor file", "text/plain")

            if u.path in STATIC:
                rel, ctype = STATIC[u.path]
                f = (HERE / rel).resolve()
                if f.exists():
                    return self._send(200, f.read_bytes(), ctype)
                return self._send(404, f"missing {rel}".encode(), "text/plain")

            self._send(404, b"not found", "text/plain")

    return Handler


def main() -> int:
    ap = argparse.ArgumentParser(description="VR/대시보드 방향 뷰어")
    ap.add_argument("--urdf", default=None, help="기본: urdf/robot_arm_temp.urdf")
    ap.add_argument("--geometry", default=None, help="링크 박스 JSON")
    ap.add_argument("--port", type=int, default=8770)
    args = ap.parse_args()

    if args.urdf:
        urdf = Path(args.urdf).resolve()
        geom = Path(args.geometry).resolve() if args.geometry else \
            HERE / f"geometry_{urdf.stem}.json"
    else:
        urdf = PKG / "urdf" / "robot_arm_temp.urdf"
        geom = HERE / "geometry_robot_arm_temp.json"

    if not urdf.exists():
        print(f"URDF 가 없습니다: {urdf}")
        return 1

    viewer = Viewer(urdf, geom)
    print(f"""
  로봇   {viewer.name}   관절 {viewer.chain.dof}개 : {', '.join(viewer.chain.joint_names)}
  EE     {viewer.chain.ee_link}
  형상   {'geometry JSON 사용' if geom and geom.exists() else '★ 형상 JSON 없음 — 정육면체로 그립니다'}

      ▶  http://localhost:{args.port}

  브라우저에서 열고 yaw / 미러 버튼을 눌러보세요.
  ★ 여기 그리는 코드(robot_view.js)는 VR 헤드셋 안에서 쓰는 것과 같은 파일입니다.

  Ctrl-C 로 종료.
""", flush=True)
    srv = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(viewer))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n  종료")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
