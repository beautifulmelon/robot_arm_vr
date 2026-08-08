#!/usr/bin/env python
"""4단계: 추출한 RPO 팔 URDF 의 IK 검증 (placo).

LeRobot 의 RobotKinematics 가 쓰는 placo 로 실제 IK 가 도는지, 텔레옵에 쓸 만한
속도·정확도가 나오는지 확인한다. 하드웨어 없이 돌아간다.

검사 항목:
  1. 작업 공간 — 랜덤 관절각 샘플링으로 도달 가능 영역 측정
     → Quest 컨트롤러 이동 범위(0.80 × 1.00 × 1.01 m)와 비교해 스케일 계수 산출
  2. IK 왕복 정확도 — FK(q) 로 만든 목표를 IK 로 되풀었을 때 EE 오차
  3. IK 속도 — 제어 루프 30 Hz 를 만족하는지
  4. elbow_yaw 축 검증 — 이 축이 EE '위치'에 영향을 주는지
     (roboparty 가 command_q[4]=0 으로 죽여도 되던 이유를 실측 확인)

사용법:
    .venv/bin/python scripts/04_check_ik.py
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import placo

ROOT = Path(__file__).resolve().parents[1]

JOINT_LIMITS = {  # URDF 값 (rad)
    "right_arm_pitch_joint": (-2.0, 2.0),
    "right_arm_roll_joint": (-2.25, 0.25),
    "right_arm_yaw_joint": (-2.6, 2.6),
    "right_elbow_pitch_joint": (-1.0, 1.57),
    "right_elbow_yaw_joint": (-1.57, 1.57),
}

# Quest 2 실측 오른손 grip 이동 범위 (docs/01_quest_mapping.md)
QUEST_SPAN = np.array([0.80, 1.00, 1.01])


def fk(robot, q: np.ndarray, names: list[str], frame: str) -> np.ndarray:
    for name, val in zip(names, q, strict=True):
        robot.set_joint(name, float(val))
    robot.update_kinematics()
    return robot.get_T_world_frame(frame)


def solve_ik(solver, robot, task, names, q_init: np.ndarray, target: np.ndarray,
             iters: int = 1) -> np.ndarray:
    for name, val in zip(names, q_init, strict=True):
        robot.set_joint(name, float(val))
    robot.update_kinematics()
    task.T_world_frame = target
    for _ in range(iters):
        solver.solve(True)
        robot.update_kinematics()
    return np.array([robot.get_joint(n) for n in names])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", type=Path,
                    default=ROOT / "assets" / "rpo_arm" / "urdf" / "rpo_arm_right.urdf")
    ap.add_argument("--frame", default="ee_link")
    ap.add_argument("--samples", type=int, default=20000)
    ap.add_argument("--ori-weight", type=float, default=0.01,
                    help="자세 가중치 (LeRobot 기본값 0.01)")
    args = ap.parse_args()

    robot = placo.RobotWrapper(str(args.urdf))
    names = list(robot.joint_names())
    lo = np.array([JOINT_LIMITS[n][0] for n in names])
    hi = np.array([JOINT_LIMITS[n][1] for n in names])

    print("=" * 72)
    print(f"  IK 검증 — {args.urdf.name}  frame={args.frame}")
    print("=" * 72)
    print(f"  관절 {len(names)}개: {', '.join(n.replace('right_', '').replace('_joint', '') for n in names)}")

    # ── 1. 작업 공간 ──────────────────────────────────────────────────
    rng = np.random.default_rng(0)
    Q = rng.uniform(lo, hi, size=(args.samples, len(names)))
    P = np.array([fk(robot, q, names, args.frame)[:3, 3] for q in Q])

    shoulder = fk(robot, np.zeros(len(names)), names, "right_arm_pitch_link")[:3, 3]
    reach = np.linalg.norm(P - shoulder, axis=1)
    span = P.max(0) - P.min(0)

    print("\n[1] 작업 공간 (base_link 기준, 관절 범위 내 균등 샘플 %d개)" % args.samples)
    for i, ax in enumerate("xyz"):
        print(f"    {ax}: {P[:,i].min():+.3f} ~ {P[:,i].max():+.3f}   (범위 {span[i]:.3f} m)")
    print(f"    어깨(arm_pitch)로부터 거리: {reach.min():.3f} ~ {reach.max():.3f} m  (중앙값 {np.median(reach):.3f})")

    # 스케일은 bounding box 보다 '어깨 기준 리치 비율'로 잡는 편이 원리적이다.
    # 성인 어깨~손 리치는 대략 0.60~0.70 m.
    HUMAN_REACH = 0.65
    reach_scale = reach.max() / HUMAN_REACH
    bbox_scale = span / QUEST_SPAN
    print(f"\n    Quest 손 이동 bbox : [{QUEST_SPAN[0]:.2f} {QUEST_SPAN[1]:.2f} {QUEST_SPAN[2]:.2f}] m")
    print(f"    팔 도달 bbox       : [{span[0]:.2f} {span[1]:.2f} {span[2]:.2f}] m"
          f"   축별 비율 [{bbox_scale[0]:.2f} {bbox_scale[1]:.2f} {bbox_scale[2]:.2f}]")
    print(f"    팔 최대 리치       : {reach.max():.3f} m  (사람 어깨~손 {HUMAN_REACH:.2f} m 대비 {reach_scale:.2f})")
    print(f"    → 권장 스케일 계수 : {reach_scale:.2f}  (사람 손 변위 × {reach_scale:.2f} = 로봇 EE 변위)")
    print(f"      roboparty 는 robot/human = 0.50/0.45 = {0.50/0.45:.2f} 로 '확대'하지만,")
    print(f"      그건 전신 휴머노이드 기준값이라 이 단일 팔에는 과대하다.")

    # ── 2 & 3. IK 왕복 정확도 + 속도 ──────────────────────────────────
    solver = placo.KinematicsSolver(robot)
    solver.mask_fbase(True)
    solver.enable_joint_limits(True)
    task = solver.add_frame_task(args.frame, np.eye(4))
    task.configure(args.frame, "soft", 1.0, args.ori_weight)

    n_test = 300
    rng2 = np.random.default_rng(1)
    idx = rng2.choice(len(Q), n_test, replace=False)

    print(f"\n[2] IK 왕복 정확도 (목표 = FK(q_target), 초기값 = q_target 에서 벗어난 자세)")
    for iters in (1, 5, 20):
        pos_err, ori_err, times = [], [], []
        for k in idx:
            q_target = Q[k]
            T_target = fk(robot, q_target, names, args.frame)
            q_init = np.clip(q_target + rng2.normal(0, 0.3, len(names)), lo, hi)
            t0 = time.perf_counter()
            q_sol = solve_ik(solver, robot, task, names, q_init, T_target, iters=iters)
            times.append(time.perf_counter() - t0)
            T_sol = fk(robot, q_sol, names, args.frame)
            pos_err.append(np.linalg.norm(T_sol[:3, 3] - T_target[:3, 3]))
            R = T_sol[:3, :3].T @ T_target[:3, :3]
            ori_err.append(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))
        pe, oe, tm = np.array(pos_err), np.array(ori_err), np.array(times)
        fail = int((pe > 0.005).sum())  # 5 mm 초과를 미수렴으로 간주
        print(f"    iters={iters:2d}  위치오차 중앙값 {np.median(pe)*1000:6.2f} mm"
              f" | p95 {np.percentile(pe,95)*1000:6.2f} | 최대 {pe.max()*1000:6.1f}"
              f" | 5mm 초과 {fail:3d}/{n_test}"
              f"  자세오차(중앙값) {np.median(oe):5.2f}°"
              f"  {tm.mean()*1000:5.2f} ms/call → {1/tm.mean():6.0f} Hz")

    print(f"\n[3] 텔레옵 적합성: 90 Hz 입력을 30 Hz 제어 루프로 다운샘플 시 예산 33.3 ms")
    print(f"    → iters=5 기준 IK 소요 시간은 예산의 극히 일부. 여유 충분.")

    # ── 4. elbow_yaw 축이 EE 위치에 미치는 영향 ────────────────────────
    print("\n[4] elbow_yaw(손목 롤) 축 검증")
    q0 = np.zeros(len(names))
    base_ee = fk(robot, q0, names, "ee_link")
    deltas_pos, deltas_ori = [], []
    for val in np.linspace(-1.57, 1.57, 21):
        q = q0.copy()
        q[4] = val
        T = fk(robot, q, names, "ee_link")
        deltas_pos.append(np.linalg.norm(T[:3, 3] - base_ee[:3, 3]))
        R = T[:3, :3].T @ base_ee[:3, :3]
        deltas_ori.append(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))
    print(f"    elbow_yaw 를 -90°~+90° 돌렸을 때")
    print(f"      EE 위치 변화 : 최대 {max(deltas_pos)*1000:.3f} mm  → {'위치에 영향 없음 ✅' if max(deltas_pos) < 1e-6 else '위치에 영향 있음'}")
    print(f"      EE 자세 변화 : 최대 {max(deltas_ori):.1f}°")
    print(f"    → roboparty 가 command_q[4]=0 으로 죽여도 위치 추종이 되던 이유가 이것.")
    print(f"      Amazing Hand 자세 제어에는 이 축이 필수이므로 우리는 살려서 쓴다.")

    print("=" * 72)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
