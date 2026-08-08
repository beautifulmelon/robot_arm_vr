#!/usr/bin/env python
"""URDF 를 등록하고 텔레옵 설정을 자동 생성한다.

새 URDF(실물 팔 모델)를 넘겨받았을 때 제일 먼저 돌리는 스크립트.
관절·한계를 파싱하고, 작업공간을 샘플링으로 측정하고, IK 가 제대로 도는지 확인한 뒤
config/arm.json 에 저장한다. 이후 모든 스크립트가 이 파일을 읽으므로 URDF 를 바꿔도
다른 코드를 손댈 필요가 없다.

사용법:
    .venv/bin/python scripts/06_setup_urdf.py --urdf path/to/my_arm.urdf
    .venv/bin/python scripts/06_setup_urdf.py --urdf my.urdf --ee-frame tool0
    .venv/bin/python scripts/06_setup_urdf.py --urdf my.urdf --list-frames   # 프레임만 확인

메시 경로가 `package://...` 로 되어 있으면 placo 가 못 읽는다. --fix-mesh-paths 로
상대 경로(`../meshes/`)로 바꿔서 복사본을 만들 수 있다.
"""

from __future__ import annotations

import argparse
import shutil
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rpo_teleop.arm_config import ArmConfig  # noqa: E402

DEFAULT_OUT = ROOT / "config" / "arm.json"


def list_frames(urdf: Path) -> int:
    import placo

    robot = placo.RobotWrapper(str(urdf))
    print(f"  구동 관절 ({len(robot.joint_names())}개):")
    for i, n in enumerate(robot.joint_names()):
        print(f"    [{i}] {n}")
    print("\n  프레임:")
    for n in robot.frame_names():
        print(f"    · {n}")
    return 0


def fix_mesh_paths(urdf: Path, out_dir: Path) -> Path:
    """package:// 나 절대경로 메시를 상대경로로 바꾸고 메시를 모아 복사한다."""
    tree = ET.parse(urdf)
    root = tree.getroot()
    out_urdf_dir = out_dir / "urdf"
    out_mesh_dir = out_dir / "meshes"
    out_urdf_dir.mkdir(parents=True, exist_ok=True)
    out_mesh_dir.mkdir(parents=True, exist_ok=True)

    found, missing = 0, []
    search_roots = [urdf.parent, urdf.parent.parent, urdf.parent.parent / "meshes"]
    for mesh in root.iter("mesh"):
        raw = mesh.get("filename", "")
        name = Path(raw.split("://")[-1]).name
        src = None
        for base in search_roots:
            for cand in (base / name, *base.rglob(name)):
                if cand.is_file():
                    src = cand
                    break
            if src:
                break
        if src:
            shutil.copy2(src, out_mesh_dir / name)
            found += 1
        else:
            missing.append(name)
        mesh.set("filename", f"../meshes/{name}")

    out = out_urdf_dir / urdf.name
    tree.write(out, encoding="utf-8", xml_declaration=True)
    print(f"  메시 {found}개 복사 → {out_mesh_dir}")
    if missing:
        print(f"  ⚠️  못 찾은 메시 {len(missing)}개: {missing[:5]}{' …' if len(missing) > 5 else ''}")
        print("     (기구학에는 영향 없지만 3D 뷰에서 해당 링크가 안 보입니다)")
    return out


def measure_position_rank(cfg: ArmConfig, n: int = 300) -> tuple[int, float, float]:
    """위치 자코비안의 랭크와 방향별 전달률을 잰다.

    관절 수가 3개여도 축이 겹치면 손끝은 2차원 곡면 위에서만 움직인다. 관절 개수로는
    안 보이고 자코비안 랭크로만 보인다. 텔레옵 체감을 좌우하므로 등록할 때 재둔다.

    Returns: (랭크, 접선 전달률, 반경 전달률)
    """
    import pinocchio as pin

    m = pin.buildModelFromUrdf(cfg.urdf_path)
    d = m.createData()
    fid = m.getFrameId(cfg.ee_frame)
    lo, hi = np.array(cfg.lower), np.array(cfg.upper)
    rng = np.random.default_rng(0)

    ranks = []
    for q in lo + (hi - lo) * rng.random((n, cfg.dof)):
        J = pin.computeFrameJacobian(m, d, q, fid, pin.LOCAL_WORLD_ALIGNED)[:3, :]
        ranks.append(int((np.linalg.svd(J, compute_uv=False) > 1e-8).sum()))
    rank = int(np.bincount(ranks).argmax())

    # 방향별 전달률: 홈에서 ±8 cm 왕복시켰을 때 손끝이 실제로 간 거리 / 지령 거리.
    #
    # 자코비안 의사역행렬 사영(J·J⁺)으로 순간값을 쓰면 안 된다. 그건 '지금 이 순간
    # 낼 수 있는 속도 비율'이라 랭크 2 인 팔에서도 22% 가 나오는데, 실제로 8 cm 를
    # 밀면 껍질에 부딪혀 2% 밖에 안 간다. 체감을 예측하려면 유한 변위로 재야 한다.
    import placo

    robot = placo.RobotWrapper(cfg.urdf_path)
    solver = placo.KinematicsSolver(robot)
    solver.mask_fbase(True)
    solver.enable_joint_limits(True)
    task = solver.add_position_task(cfg.ee_frame, np.zeros(3))
    task.configure("p", "soft", 10000.0)

    def home_pose():
        for name, val in zip(cfg.joint_names, cfg.home, strict=True):
            robot.set_joint(name, float(val))
        robot.update_kinematics()
        return robot.get_T_world_frame(cfg.ee_frame)[:3, 3].copy()

    p0 = home_pose()
    radial = p0 - cfg.shoulder
    radial = radial / max(np.linalg.norm(radial), 1e-9)
    tang = np.cross(radial, [0.0, 0.0, 1.0])
    tang = tang / max(np.linalg.norm(tang), 1e-9)

    def transmitted(direction, amp=0.08, steps=60):
        """지령 방향 d 로 ±amp 왕복시켰을 때, 손끝이 d 방향으로 간 폭 / 지령 폭.

        총 경로 길이로 재면 안 된다. 랭크 2 인 팔은 반경 방향을 밀면 껍질을 따라
        옆으로 미끄러지는데, 경로 길이로는 그게 '전달됐다'로 잡혀 25% 가 나온다.
        실제로 d 방향으로 간 거리를 봐야 한다.
        """
        home_pose()
        along = []
        for k in range(1, steps + 1):
            off = amp * np.sin(2 * np.pi * k / steps)
            # 실제 텔레옵 경로와 같게 클램프를 거친다 (ArmIK.solve 가 하는 일)
            target, _ = cfg.clamp_position(p0 + off * direction)
            task.target_world = target
            for _ in range(5):
                solver.solve(True)
                robot.update_kinematics()
            p = robot.get_T_world_frame(cfg.ee_frame)[:3, 3]
            along.append(float((p - p0) @ direction))
        return (max(along) - min(along)) / (2 * amp)

    return rank, transmitted(tang), transmitted(radial)


def benchmark_ik(cfg: ArmConfig, n: int = 200, iters: int = 5, arbitrary_pose: bool = False) -> dict:
    """IK 정확도·속도 측정.

    Args:
        arbitrary_pose:
            False — 목표를 FK(랜덤 관절각)로 만든다. 정의상 도달 가능한 6-DOF 자세이므로
                    **솔버 품질**을 잰다. 자유도가 모자라도 오차가 0으로 나온다.
            True  — 작업공간 안의 임의 위치 + 임의 자세를 목표로 준다. 대부분 도달 불가능한
                    조합이므로 **자유도 충분성**을 잰다. 5-DOF 팔은 여기서 자세오차가 크게 뜬다.
    """
    import placo

    robot = placo.RobotWrapper(cfg.urdf_path)
    solver = placo.KinematicsSolver(robot)
    solver.mask_fbase(True)
    solver.enable_joint_limits(True)
    pos_task = solver.add_position_task(cfg.ee_frame, np.zeros(3))
    pos_task.configure("p", "soft", 10000.0)
    ori_task = solver.add_orientation_task(cfg.ee_frame, np.eye(3))
    ori_task.configure("o", "soft", 1.0)

    def set_q(q):
        for name, val in zip(cfg.joint_names, q, strict=True):
            robot.set_joint(name, float(val))
        robot.update_kinematics()

    rng = np.random.default_rng(1)
    lo, hi = np.array(cfg.lower), np.array(cfg.upper)
    pos_err, ori_err, times = [], [], []

    ws_lo, ws_hi = np.array(cfg.workspace_min), np.array(cfg.workspace_max)
    for _ in range(n):
        q_t = rng.uniform(lo, hi)
        set_q(q_t)
        T = robot.get_T_world_frame(cfg.ee_frame).copy()
        if arbitrary_pose:
            # 작업공간 안 임의 위치 + 임의 자세. 대부분 도달 불가능한 조합이다.
            pos = rng.uniform(ws_lo, ws_hi)
            pos, _ = cfg.clamp_position(pos)
            rv = rng.normal(0, 1, 3)
            rv = rv / np.linalg.norm(rv) * rng.uniform(0, np.pi)
            th = np.linalg.norm(rv)
            k = rv / th
            K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
            T = np.eye(4)
            T[:3, :3] = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)
            T[:3, 3] = pos
        set_q(np.clip(q_t + rng.normal(0, 0.3, cfg.dof), lo, hi))

        pos_task.target_world = T[:3, 3]
        ori_task.R_world_frame = T[:3, :3]
        t0 = time.perf_counter()
        try:
            for _ in range(iters):
                solver.solve(True)
                robot.update_kinematics()
        except Exception:
            continue
        times.append(time.perf_counter() - t0)
        ee = robot.get_T_world_frame(cfg.ee_frame)
        pos_err.append(np.linalg.norm(ee[:3, 3] - T[:3, 3]))
        R = ee[:3, :3].T @ T[:3, :3]
        ori_err.append(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))

    return {
        "n": len(pos_err),
        "pos_median_mm": float(np.median(pos_err) * 1000) if pos_err else float("nan"),
        "pos_p95_mm": float(np.percentile(pos_err, 95) * 1000) if pos_err else float("nan"),
        "ori_median_deg": float(np.median(ori_err)) if ori_err else float("nan"),
        "ms_per_call": float(np.mean(times) * 1000) if times else float("nan"),
    }


def benchmark_tracking(cfg: ArmConfig, steps: int = 500, max_step_m: float = 0.05) -> dict:
    """연속 궤적 추종 — 실제 텔레옵과 같은 조건.

    ①②는 목표를 매번 '점프'시키는 인위적 조건이다. 실제 텔레옵은 홈 자세에서 출발해
    매 프레임 목표가 조금씩(레이트 리밋 max_step_m 이내) 움직인다. 이 지표가 체감
    품질을 가장 잘 예측한다.
    """
    import placo

    robot = placo.RobotWrapper(cfg.urdf_path)
    solver = placo.KinematicsSolver(robot)
    solver.mask_fbase(True)
    solver.enable_joint_limits(True)
    # dt 를 프레임당 solve 횟수로 나눈다. placo 는 solve() 한 번을 한 스텝으로 보고
    # dt × 속도한계 만큼만 움직이므로, 안 나누면 프레임당 이동량이 그 횟수만큼 커진다
    # (실측: 5회에서 허용 7.20°/프레임 대비 36.00°). 05_teleop_sim.py 와 같은 규칙.
    ITERS = 5
    solver.dt = cfg.control_dt / ITERS
    solver.enable_velocity_limits(True)  # 관절 한계 ±180° 에서의 360° 감김 방지
    pos_task = solver.add_position_task(cfg.ee_frame, np.zeros(3))
    pos_task.configure("p", "soft", 10000.0)
    ori_task = solver.add_orientation_task(cfg.ee_frame, np.eye(3))
    ori_task.configure("o", "soft", 1.0)

    for name, val in zip(cfg.joint_names, cfg.home, strict=True):
        robot.set_joint(name, float(val))
    robot.update_kinematics()
    home = robot.get_T_world_frame(cfg.ee_frame).copy()

    # 홈 EE 주변을 도는 리사주 궤적. 진폭은 궤적 전체가 [min_reach, max_reach] 안에
    # 들어오도록 잡는다. 벗어나면 클램프가 걸려 '팔 성능'이 아니라 '클램프 거동'을
    # 재게 된다 (실측: 진폭이 커서 목표가 min_reach 아래로 내려가면 p95 26mm).
    home_r = float(np.linalg.norm(home[:3, 3] - cfg.shoulder))
    amp = min(cfg.max_reach * 0.25,
              max(0.02, (home_r - cfg.min_reach) * 0.85),
              max(0.02, (cfg.max_reach - home_r) * 0.85))
    cur = home[:3, 3].copy()
    errs = []
    for i in range(steps):
        t = i * 0.02
        goal = home[:3, 3] + amp * np.array([np.sin(t), np.sin(1.3 * t), 0.5 * np.sin(0.7 * t)])
        goal, _ = cfg.clamp_position(goal)
        step = goal - cur
        n = float(np.linalg.norm(step))
        cur = goal if n <= max_step_m else cur + step * (max_step_m / n)
        pos_task.target_world = cur
        ori_task.R_world_frame = home[:3, :3]
        for _ in range(ITERS):
            solver.solve(True)
            robot.update_kinematics()
        errs.append(float(np.linalg.norm(robot.get_T_world_frame(cfg.ee_frame)[:3, 3] - cur)))

    e = np.array(errs[50:])  # 초기 수렴 구간 제외
    return {"median_mm": float(np.median(e) * 1000),
            "p95_mm": float(np.percentile(e, 95) * 1000),
            "max_mm": float(e.max() * 1000)}


def main() -> int:
    ap = argparse.ArgumentParser(description="URDF 등록 및 텔레옵 설정 생성")
    ap.add_argument("--urdf", type=Path, required=True)
    ap.add_argument("--ee-frame", type=str, default=None,
                    help="엔드이펙터 프레임 이름 (기본: 자동 탐색)")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--samples", type=int, default=20000, help="작업공간 샘플 수")
    ap.add_argument("--human-reach", type=float, default=0.65,
                    help="조작자 어깨~손 리치 (m). 스케일 계수 산출에 쓴다")
    ap.add_argument("--list-frames", action="store_true", help="관절/프레임만 출력하고 종료")
    ap.add_argument("--fix-mesh-paths", action="store_true",
                    help="package:// 메시 경로를 상대경로로 바꾼 복사본을 만든다")
    args = ap.parse_args()

    if not args.urdf.exists():
        print(f"❌ URDF 없음: {args.urdf}", file=sys.stderr)
        return 1

    urdf = args.urdf.resolve()
    print("=" * 74)
    print(f"  URDF 등록 — {urdf.name}")
    print("=" * 74)

    if args.fix_mesh_paths:
        urdf = fix_mesh_paths(urdf, ROOT / "assets" / urdf.stem)
        print(f"  정리된 URDF: {urdf.relative_to(ROOT)}")

    if args.list_frames:
        return list_frames(urdf)

    try:
        cfg = ArmConfig.from_urdf(urdf, ee_frame=args.ee_frame, samples=args.samples)
    except Exception as exc:
        print(f"❌ 설정 생성 실패: {exc}", file=sys.stderr)
        print("\n  프레임 목록을 보려면: --list-frames", file=sys.stderr)
        return 1
    cfg.human_reach = args.human_reach

    ws_min, ws_max = np.array(cfg.workspace_min), np.array(cfg.workspace_max)
    span = ws_max - ws_min

    print(f"\n  자유도    : {cfg.dof}")
    print(f"  EE 프레임 : {cfg.ee_frame}")
    print(f"  base      : {cfg.base_frame}")
    print(f"  총 질량   : {cfg.total_mass:.3f} kg")
    print(f"\n  관절:")
    for i, (n, lo, hi, eff, vel) in enumerate(
        zip(cfg.joint_names, cfg.lower, cfg.upper, cfg.effort, cfg.velocity, strict=True)
    ):
        print(f"    [{i}] {n:30s} [{lo:+.3f}, {hi:+.3f}] rad "
              f"({np.degrees(lo):+7.1f}° ~ {np.degrees(hi):+7.1f}°)  "
              f"τ={eff:.0f} ω={vel:.2f}")

    print(f"\n  작업 공간 ({args.samples} 샘플):")
    for i, ax in enumerate("xyz"):
        print(f"    {ax}: {ws_min[i]:+.3f} ~ {ws_max[i]:+.3f}   (폭 {span[i]:.3f} m)")
    print(f"    어깨 위치  : [{cfg.shoulder[0]:+.3f} {cfg.shoulder[1]:+.3f} {cfg.shoulder[2]:+.3f}]")
    print(f"    리치 범위  : {cfg.min_reach:.3f} ~ {cfg.max_reach:.3f} m (안전 마진 반영)")
    print(f"\n  홈 자세 (특이점 회피):")
    print(f"    q(deg) = [{' '.join(f'{np.degrees(v):+7.1f}' for v in cfg.home)}]")
    cond_mark = "✅" if cfg.home_condition < 10 else ("⚠️ " if cfg.home_condition < 100 else "❌")
    print(f"    자코비안 조건수 {cfg.home_condition:.1f} {cond_mark}  (작을수록 좋음, 10 이하 권장)")
    print(f"    스케일 계수: {cfg.position_scale:.3f}  "
          f"(= 최대리치 {cfg.max_reach:.3f} / 사람리치 {cfg.human_reach:.2f})")

    rank, tang_t, rad_t = measure_position_rank(cfg)
    cfg.position_rank = rank
    print(f"\n  위치 자유도 (자코비안 랭크): {rank}/3 " + ("✅" if rank >= 3 else "❌"))
    print(f"    방향별 전달률 — 껍질 따라 {tang_t*100:.0f}%   안팎으로 {rad_t*100:.0f}%")
    if rank < 3:
        print("    ❌ 손끝이 곡면 위에서만 움직입니다. 관절 수와 무관한 구조적 한계입니다.")
        print("       텔레옵에서 안팎으로 밀면 반응이 없습니다 (고장 아님).")
        print("       가리키는 방향 조종은 정상 동작합니다.")

    print("\n  IK 검증 (position soft 10000 : orientation soft 1, iters=5)")
    solver_b = benchmark_ik(cfg, arbitrary_pose=False)
    print(f"    ① 솔버 품질  — 목표를 FK 로 생성(반드시 도달 가능)")
    print(f"       성공 {solver_b['n']}/200  위치 {solver_b['pos_median_mm']:.3f} mm "
          f"(p95 {solver_b['pos_p95_mm']:.3f})  자세 {solver_b['ori_median_deg']:.2f}°  "
          f"{solver_b['ms_per_call']:.3f} ms/call → {1000/solver_b['ms_per_call']:.0f} Hz")

    # ②는 랜덤 자세에서 랜덤 목표로 '점프'시키는 최악 조건이라 iters=5 로는 수렴 전이가
    # 섞인다. 자유도 한계만 보려면 충분히 수렴시켜야 한다 (실측: 위치오차는 iters 를
    # 늘리면 180→20 mm 로 줄지만 자세오차는 94→85° 로 거의 안 준다 = 이쪽이 진짜 한계).
    dof_b = benchmark_ik(cfg, arbitrary_pose=True, iters=100)
    print("    ② 자유도 충분성 — 작업공간 내 임의 위치 + 임의 자세 (충분히 수렴, iters=100)")
    print(f"       위치 {dof_b['pos_median_mm']:.1f} mm  자세 {dof_b['ori_median_deg']:.1f}°")

    if cfg.dof < 6:
        print(f"\n    ℹ️  {cfg.dof}-DOF 는 6-DOF 목표를 다 맞출 수 없습니다 (자유도 {6-cfg.dof} 부족).")
        print(f"       ②의 자세오차 {dof_b['ori_median_deg']:.0f}° 가 그 한계이며 버그가 아닙니다.")
        print(f"       위치를 우선(가중치 10000:1)하므로 손끝은 정확히 갑니다 — ① 참고.")
        print(f"       실제 텔레옵은 매 프레임 목표가 조금씩만 움직이므로 ①에 가깝게 동작합니다.")
    else:
        print(f"\n    ✅ {cfg.dof}-DOF — 임의 6-DOF 자세도 자세오차 "
              f"{dof_b['ori_median_deg']:.1f}° 로 추종 가능합니다.")
    bench = solver_b

    track = benchmark_tracking(cfg)
    print("    ③ 연속 추종 — 홈에서 출발, 프레임당 5 cm 이내 이동 (실제 텔레옵 조건)")
    print(f"       위치오차 중앙값 {track['median_mm']:.3f} mm  p95 {track['p95_mm']:.2f}  "
          f"최대 {track['max_mm']:.2f} mm")
    if track["p95_mm"] < 5:
        print("       ✅ 텔레옵에 충분합니다.")
    else:
        print("       ⚠️  오차가 큽니다. 홈 자세 조건수와 리치 설정을 확인하세요.")

    out = cfg.save(args.out)
    # --out 을 상대경로로 주면 relative_to 가 터진다. 절대경로로 맞춘 뒤 비교한다.
    out_abs = Path(out).resolve()
    shown = out_abs.relative_to(ROOT) if out_abs.is_relative_to(ROOT) else out_abs
    print(f"\n  💾 저장: {shown}")
    print("=" * 74)
    print("  다음 단계:")
    if out_abs.name == "arm_temp.json":
        print("    ./run.sh --temp      # 임시 팔로 텔레옵")
    else:
        print("    ./run.sh             # 이 설정으로 바로 텔레옵")
    print("=" * 74)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
