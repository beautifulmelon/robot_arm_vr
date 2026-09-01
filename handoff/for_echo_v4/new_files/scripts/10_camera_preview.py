#!/usr/bin/env python
"""본체 카메라(C920) 시점을 기울기별로 렌더한다 — "바닥이 어디부터 보이나".

    .venv/bin/python scripts/10_camera_preview.py
    .venv/bin/python scripts/10_camera_preview.py --tilts 0 30 60 --out out/cam

왜 직접 그리는가
    "무엇이 화면에 들어오는가" 는 순수 기하 문제라 사실적인 렌더가 필요 없다.
    핀홀 모델로 투영해서 선만 그으면 **정확한 답**이 나온다. 오히려 조명·질감이
    없어서 시야 경계가 또렷하게 보인다.

    ★ 눈으로 "잘 보이네" 하고 넘어가면 안 되는 종류라서, 화면에 격자 거리와
      각도를 같이 찍는다.

카메라 규약
    프레임 body_camera_mount 는 ROS optical (+Z 광축, +Y 아래, +X 오른쪽).
    기울기 θ 는 그 광축을 **아래로** θ 만큼 내린 것이다.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rpo_teleop.arm_config import ArmConfig  # noqa: E402
from rpo_teleop.arm_visual import extract_link_boxes  # noqa: E402

BODY_URDF = ROOT / "assets/body/body_with_arm_nomimic.urdf"
BODY_JSON = ROOT / "web/vendor/body_geometry.json"

# 62_CAMERA_SPEC.txt 의 값
CAMS = {
    "body":  {"frame": "body_camera_mount", "hfov": 70.4, "vfov": 43.3, "name": "C920 (본체)"},
    "wrist": {"frame": "camera_mount",      "hfov": 69.0, "vfov": 42.0, "name": "D435i RGB (손목)"},
}
W, H = 1280, 720

COL = {
    "floor": (60, 70, 80), "floor_hi": (90, 110, 130),
    "body": (110, 120, 130), "arm": (230, 150, 60),
    "box": (240, 70, 60), "reach": (70, 200, 120), "text": (220, 225, 230),
}


class Cam:
    """핀홀 카메라. tilt 는 광축을 아래로 내린 각(도)."""

    def __init__(self, pos, tilt_deg, hfov=70.4, vfov=43.3, R=None):
        self.p = np.asarray(pos, dtype=float)
        if R is not None:
            # ★ 프레임이 주어지면 그대로 쓴다 (ROS optical: +Z 광축, +Y 아래, +X 오른쪽).
            #   손목 카메라는 팔을 따라 움직이므로 기울기를 따로 주지 않는다.
            self.fwd, self.right, self.down = R[:, 2], R[:, 0], R[:, 1]
        else:
            t = np.radians(tilt_deg)
            self.fwd = np.array([np.cos(t), 0.0, -np.sin(t)])
            self.right = np.array([0.0, -1.0, 0.0])
            self.down = np.array([-np.sin(t), 0.0, -np.cos(t)])
        self.tx = np.tan(np.radians(hfov) / 2)
        self.ty = np.tan(np.radians(vfov) / 2)

    def project(self, p):
        """월드 점 → (u, v, z). z<=0 이면 카메라 뒤."""
        v = np.asarray(p, dtype=float) - self.p
        z = v @ self.fwd
        if z <= 1e-6:
            return None, None, z
        return (W / 2 * (1 + (v @ self.right) / z / self.tx),
                H / 2 * (1 + (v @ self.down) / z / self.ty), z)

    def seg(self, a, b):
        """선분을 근평면에서 자른 뒤 화면 좌표로. 둘 다 뒤면 None."""
        va, vb = np.asarray(a, float) - self.p, np.asarray(b, float) - self.p
        za, zb = va @ self.fwd, vb @ self.fwd
        eps = 1e-3
        if za <= eps and zb <= eps:
            return None
        if za <= eps or zb <= eps:
            s = (eps - za) / (zb - za)
            if za <= eps:
                a = np.asarray(a, float) + s * (np.asarray(b, float) - np.asarray(a, float))
            else:
                b = np.asarray(a, float) + s * (np.asarray(b, float) - np.asarray(a, float))
        ua, vaa, _ = self.project(a)
        ub, vbb, _ = self.project(b)
        if ua is None or ub is None:
            return None
        return (ua, vaa, ub, vbb)


def box_edges(center, size, R=np.eye(3)):
    """상자 12개 모서리를 (a,b) 쌍으로."""
    h = np.asarray(size, float) / 2
    c = np.asarray(center, float)
    pts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                pts.append(c + R @ (np.array([sx, sy, sz]) * h))
    idx = [(0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3), (2, 6),
           (3, 7), (4, 5), (4, 6), (5, 7), (6, 7)]
    return [(pts[i], pts[j]) for i, j in idx]


def quat_to_mat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def arm_state(q5):
    """팔을 q5 자세로 놓고 (링크 상자들, 프레임 pose) 를 돌려준다."""
    import placo
    r = placo.RobotWrapper(str(BODY_URDF))
    names = list(r.joint_names())
    cfg = ArmConfig.load(ROOT / "config/arm.json")
    q = dict.fromkeys(names, 0.0)
    for n, v in zip(cfg.joint_names, q5, strict=True):
        q[n] = float(v)
    for n, v in q.items():
        r.set_joint(n, v)
    r.update_kinematics()
    boxes = extract_link_boxes(str(BODY_URDF))
    segs = []
    for lname, b in boxes.items():
        if lname == "body_base":
            continue                      # 본체는 따로 그린다
        try:
            T = r.get_T_world_frame(lname)
        except Exception:
            continue
        segs += box_edges(T[:3, 3] + T[:3, :3] @ np.asarray(b["center"]),
                          b["size"], T[:3, :3])
    return segs, r


def static_scene(box_xyz, box_mm):
    """팔을 뺀 나머지 — 바닥·도달영역·본체·박스. 자세와 무관하다."""
    out = []
    g = []
    for x in np.arange(-0.4, 2.41, 0.2):
        g.append((np.array([x, -1.2, 0]), np.array([x, 1.2, 0])))
    for y in np.arange(-1.2, 1.21, 0.2):
        g.append((np.array([-0.4, y, 0]), np.array([2.4, y, 0])))
    out.append((COL["floor"], g, 1))

    cs = [np.array([0.55, -0.30, 0.002]), np.array([0.75, -0.30, 0.002]),
          np.array([0.75, 0.30, 0.002]), np.array([0.55, 0.30, 0.002])]
    out.append((COL["reach"], [(cs[i], cs[(i + 1) % 4]) for i in range(4)], 2))

    data = json.loads(BODY_JSON.read_text())
    bb = []
    for p in data["parts"]:
        bb += box_edges(p["center"], p["size"], quat_to_mat(p["quat"]))
    out.append((COL["body"], bb, 1))

    s = box_mm / 1000.0
    out.append((COL["box"], box_edges(np.asarray(box_xyz) / 1000.0, [s, s, s]), 3))
    return out


def draw(cam, items, path, header, footer, marks=()):
    img = Image.new("RGB", (W, H), (14, 17, 23))
    d = ImageDraw.Draw(img)
    for color, segs, wdt in items:
        for a, b in segs:
            sg = cam.seg(a, b)
            if sg:
                d.line(sg, fill=color, width=wdt)
    for p3, col in marks:
        u, v, z = cam.project(p3)
        if u is not None and 0 <= u < W and 0 <= v < H:
            d.line((u - 14, v, u + 14, v), fill=col, width=2)
            d.line((u, v - 14, u, v + 14), fill=col, width=2)
    d.rectangle((0, 0, W, 20 + 20 * len(header)), fill=(20, 24, 32))
    for k, (txt, col) in enumerate(header):
        d.text((16, 10 + 20 * k), txt, fill=col)
    d.rectangle((0, H - 26, W, H), fill=(20, 24, 32))
    d.text((16, H - 20), footer, fill=(150, 160, 170))
    img.save(path)


def in_frame(cam, p3):
    u, v, z = cam.project(p3)
    return u is not None and 0 <= u < W and 0 <= v < H


def main() -> int:
    ap = argparse.ArgumentParser(description="카메라 시점 렌더 (본체 C920 / 손목 D435i)")
    ap.add_argument("--camera", choices=("body", "wrist"), default="body")
    ap.add_argument("--tilts", type=float, nargs="+", default=[0, 20, 30, 40, 50, 60, 70],
                    help="body 카메라 전용 — 아래로 내릴 각도들")
    ap.add_argument("--heights", type=float, nargs="+", default=[300, 200, 120, 60],
                    help="wrist 카메라 전용 — 박스 위 몇 mm 에서 볼지")
    ap.add_argument("--box", type=float, nargs=3, default=[600, 0, 35],
                    metavar=("X", "Y", "Z"))
    ap.add_argument("--box-mm", type=float, default=70.0)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    import placo
    from rpo_teleop.transforms import rotvec_to_rotation
    import importlib.util
    spec = importlib.util.spec_from_file_location("t", ROOT / "scripts/05_teleop_sim.py")
    tele = importlib.util.module_from_spec(spec)
    sys.argv = ["x"]
    try:
        spec.loader.exec_module(tele)
    except SystemExit:
        pass

    cfg = ArmConfig.load(ROOT / "config/arm.json")
    spec_cam = CAMS[args.camera]
    out = args.out or (ROOT / f"out/camera_{args.camera}")
    out.mkdir(parents=True, exist_ok=True)
    box = np.asarray(args.box, float)
    statics = static_scene(args.box, args.box_mm)
    SH = np.array([314.38, -0.02, 184.46])          # 팔 어깨 (본체 좌표)

    if args.camera == "body":
        segs, r = arm_state(cfg.home)
        items = statics + [(COL["arm"], segs, 1)]
        pc = r.get_T_world_frame(spec_cam["frame"])[:3, 3]
        for t in args.tilts:
            cam = Cam(pc, t, spec_cam["hfov"], spec_cam["vfov"])
            gx = pc[0] + pc[2] / np.tan(np.radians(t)) if t > 0.01 else None
            note = (f"광축이 바닥에 닿는 곳: 본체 앞 {gx*1000:.0f} mm" if gx
                    else "광축이 바닥과 만나지 않음 (수평)")
            seen = in_frame(cam, box / 1000.0)
            p = out / f"tilt_{int(t):02d}.png"
            draw(cam, items, p,
                 [(f"{spec_cam['name']}  아래로 {t:.0f}°   "
                   f"HFOV {spec_cam['hfov']}° x VFOV {spec_cam['vfov']}°   {W}x{H}", COL["text"]),
                  (note, (150, 200, 255)),
                  (f"테스트 박스: {'화면 안 ✅' if seen else '화면 밖 ❌'}",
                   COL["reach"] if seen else COL["box"])],
                 "회색=본체  주황=팔(홈자세)  빨강=박스  초록=바닥 도달영역  격자 200mm",
                 [([gx, 0, 0], (120, 200, 255))] if gx else [])
            print(f"  {t:5.0f}°  →  {p.relative_to(ROOT)}   박스 {'안 ✅' if seen else '밖'}")
        return 0

    # ── 손목 카메라: 박스에 접근하는 자세들 ──────────────────────────
    ik = tele.ArmIK(cfg)
    R_down = rotvec_to_rotation(np.array([np.pi, 0, 0]))
    poses = [("home", cfg.home)]
    for hmm in args.heights:
        tgt_body = np.array([box[0], box[1], box[2] + hmm])
        p_arm = (tgt_body - SH) / 1000.0
        ik.set_q(cfg.home)
        T = np.eye(4); T[:3, 3] = p_arm; T[:3, :3] = R_down
        for _ in range(400):
            q, _ = ik.solve(T)
        err = np.linalg.norm(ik.fk()[:3, 3] - p_arm) * 1000
        poses.append((f"h{int(hmm):03d}", q.copy()))
        print(f"  박스 위 {hmm:4.0f}mm  IK 오차 {err:5.1f}mm")

    for label, q5 in poses:
        segs, r = arm_state(q5)
        Tc = r.get_T_world_frame(spec_cam["frame"])
        cam = Cam(Tc[:3, 3], 0, spec_cam["hfov"], spec_cam["vfov"], R=Tc[:3, :3])
        hm = r.get_T_world_frame("hand_mount")[:3, 3]
        seen_box = in_frame(cam, box / 1000.0)
        seen_hand = in_frame(cam, hm)
        dist = np.linalg.norm(box / 1000.0 - Tc[:3, 3]) * 1000
        p = out / f"wrist_{label}.png"
        draw(cam, statics + [(COL["arm"], segs, 1)], p,
             [(f"{spec_cam['name']}   HFOV {spec_cam['hfov']}° x VFOV {spec_cam['vfov']}°   {W}x{H}",
               COL["text"]),
              (f"자세: {label}   카메라→박스 {dist:.0f} mm", (150, 200, 255)),
              (f"박스 {'안 ✅' if seen_box else '밖 ❌'}   ·   "
               f"손끝(hand_mount) {'안 ✅' if seen_hand else '밖 ❌'}",
               COL["reach"] if (seen_box and seen_hand) else COL["box"])],
             "회색=본체  주황=팔·손  빨강=박스  초록=바닥 도달영역  격자 200mm",
             [(box / 1000.0, (120, 200, 255))])
        print(f"  {label:6s} → {p.relative_to(ROOT)}  박스 {'안' if seen_box else '밖'} "
              f"/ 손 {'안' if seen_hand else '밖'}  (거리 {dist:.0f}mm)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
