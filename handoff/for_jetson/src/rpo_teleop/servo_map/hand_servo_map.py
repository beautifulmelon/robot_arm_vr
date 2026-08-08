#!/usr/bin/env python3
"""AmazingHand 서보각 <-> 관절각 변환.

왜 이게 필요한가
----------------
실물 AmazingHand 는 손가락마다 서보 2개가 평행 4절 + 차동 기구를 통해
관절 3개(외전 A / 굴곡 B / 원위 C)를 움직인다. 서보 2개로 3관절이 결정되므로
관절각을 독립적으로 명령하면 실물에서 나올 수 없는 자세가 만들어진다.

  서보 정사각형 [-90,90]^2 의 (A,B) 상이 외접 사각형에서 차지하는 비율 = 45.3%
  -> URDF 의 독립 관절 리밋(사각형)의 절반 이상이 도달 불가 영역

그래서 제어/학습의 액션 공간을 '서보각'으로 두고, 시뮬레이터에는 이 모듈로
변환한 관절각을 넣는다. 그러면 실물과 액션 공간이 완전히 같아지고
도달 불가 조합이 원천적으로 사라진다.

  정책/텔레오퍼레이션 --(서보각 8개)--> ┬-> 실물 SCS0009 (그대로)
                                      └-> servo_to_joint() -> Isaac 관절 목표

사용
----
    import numpy as np
    from hand_servo_map import servo_to_joint, joint_to_servo, SERVO_LIMIT_RAD

    # 순변환: 서보각 -> 관절각
    A, B, C = servo_to_joint(theta1, theta2)          # rad, 배열 가능

    # 역변환: (외전, 굴곡) -> 서보각   (텔레오퍼레이션용)
    t1, t2 = joint_to_servo(A_target, B_target)

torch 텐서를 넣으면 torch 로 계산한다 (Isaac Lab 에서 수천 환경 벡터화 가능).

계수 출처
---------
    hand_servo_poly2d.json  <- hand_servo_table.py 로 MuJoCo 원본 모델을
                               전 구간 스윕해 적합한 2변수 4차 다항식
    손가락 4개는 기구적으로 동일하므로(편차 < 0.002 deg) 계수 하나를 공유한다.
"""
import json
from pathlib import Path

import numpy as np

_HERE = Path(__file__).parent
_COEF_PATH = _HERE / "hand_servo_poly2d.json"

SERVO_LIMIT_RAD = np.pi / 2          # SCS0009 정격 ±90 deg
N_FINGERS = 4


def _load():
    d = json.loads(_COEF_PATH.read_text())
    out = {}
    for k in ("A", "B", "C"):
        out[k] = (np.array(d[k]["terms_ic_jd"], dtype=np.int64),
                  np.array(d[k]["coeff"], dtype=np.float64))
    return out


_C = _load()


def _is_torch(x):
    return type(x).__module__.startswith("torch")


def _poly2d(tc, td, terms, coeff):
    """sum_k coeff[k] * tc^i_k * td^j_k"""
    if _is_torch(tc):
        import torch
        cf = torch.as_tensor(coeff, dtype=tc.dtype, device=tc.device)
        out = torch.zeros_like(tc)
        for k in range(len(coeff)):
            i, j = int(terms[k, 0]), int(terms[k, 1])
            out = out + cf[k] * tc.pow(i) * td.pow(j)
        return out
    out = np.zeros_like(np.asarray(tc, dtype=float))
    for k in range(len(coeff)):
        i, j = terms[k]
        out = out + coeff[k] * tc ** i * td ** j
    return out


def servo_to_joint(theta1, theta2):
    """서보각 -> 관절각.  전부 rad.  numpy/torch/스칼라 모두 가능.

    반환 (A, B, C) = (외전, 굴곡, 원위)
    """
    tc = (theta1 + theta2) / 2.0
    td = (theta1 - theta2) / 2.0
    return tuple(_poly2d(tc, td, *_C[k]) for k in ("A", "B", "C"))


def servo_to_joint_flat(theta):
    """서보각 8개(손가락1 m1,m2, 손가락2 m1,m2, ...) -> 관절각 12개.

    반환 순서는 URDF 조인트 순서와 같다:
        [f1_abduction, f1_flexion, f1_distal, f2_abduction, ...]
    theta shape (..., 8) -> (..., 12)
    """
    if _is_torch(theta):
        import torch
        outs = []
        for f in range(N_FINGERS):
            A, B, C = servo_to_joint(theta[..., 2 * f], theta[..., 2 * f + 1])
            outs += [A, B, C]
        return torch.stack(outs, dim=-1)
    theta = np.asarray(theta, dtype=float)
    outs = []
    for f in range(N_FINGERS):
        A, B, C = servo_to_joint(theta[..., 2 * f], theta[..., 2 * f + 1])
        outs += [A, B, C]
    return np.stack(outs, axis=-1)


def joint_to_servo(A_target, B_target, iters=40, tol=1e-9):
    """(외전, 굴곡) -> 서보각.  뉴턴법 역변환.  전부 rad, numpy 전용.

    C 는 서보로 결정되므로 입력하지 않는다.
    도달 불가한 목표를 넣으면 서보 리밋에 물린 채로 수렴한다 (클램프됨).
    """
    A_t = np.asarray(A_target, dtype=float)
    B_t = np.asarray(B_target, dtype=float)
    tc = np.zeros_like(A_t)
    td = np.zeros_like(B_t)
    h = 1e-6
    for _ in range(iters):
        A0 = _poly2d(tc, td, *_C["A"])
        B0 = _poly2d(tc, td, *_C["B"])
        rA, rB = A0 - A_t, B0 - B_t
        if np.max(np.abs(rA)) < tol and np.max(np.abs(rB)) < tol:
            break
        dAc = (_poly2d(tc + h, td, *_C["A"]) - A0) / h
        dAd = (_poly2d(tc, td + h, *_C["A"]) - A0) / h
        dBc = (_poly2d(tc + h, td, *_C["B"]) - B0) / h
        dBd = (_poly2d(tc, td + h, *_C["B"]) - B0) / h
        det = dAc * dBd - dAd * dBc
        det = np.where(np.abs(det) < 1e-12, 1e-12, det)
        tc = tc - (rA * dBd - rB * dAd) / det
        td = td - (dAc * rB - dBc * rA) / det
        # 서보 리밋 안으로
        t1 = np.clip(tc + td, -SERVO_LIMIT_RAD, SERVO_LIMIT_RAD)
        t2 = np.clip(tc - td, -SERVO_LIMIT_RAD, SERVO_LIMIT_RAD)
        tc, td = (t1 + t2) / 2, (t1 - t2) / 2
    return tc + td, tc - td


def is_reachable(A_target, B_target, tol_deg=0.5):
    """(A,B) 조합이 서보로 도달 가능한가."""
    t1, t2 = joint_to_servo(A_target, B_target)
    A, B, _ = servo_to_joint(t1, t2)
    e = np.maximum(np.abs(A - A_target), np.abs(B - B_target))
    return e <= np.radians(tol_deg)


if __name__ == "__main__":
    print(__doc__.strip().splitlines()[0])
    print(f"\n계수 파일: {_COEF_PATH.name}")
    for k in ("A", "B", "C"):
        print(f"  {k}: 항 {len(_C[k][1])}개")

    print("\n■ 왕복 검사 (서보 -> 관절 -> 서보)")
    rng = np.random.default_rng(0)
    t1 = rng.uniform(-SERVO_LIMIT_RAD, SERVO_LIMIT_RAD, 2000)
    t2 = rng.uniform(-SERVO_LIMIT_RAD, SERVO_LIMIT_RAD, 2000)
    A, B, C = servo_to_joint(t1, t2)
    r1, r2 = joint_to_servo(A, B)
    e = np.degrees(np.maximum(np.abs(r1 - t1), np.abs(r2 - t2)))
    print(f"   서보 복원 오차  RMS {np.sqrt((e**2).mean()):.4f} deg   최대 {e.max():.4f} deg")

    print("\n■ 도달 가능 판정 예시")
    for a, b, lbl in ((0, 40, "굴곡만 크게"), (30, 0, "외전만 크게"),
                      (30, -50, "외전+굴곡 동시(불가 예상)"), (10, 10, "완만")):
        ok = bool(is_reachable(np.radians(a), np.radians(b)))
        print(f"   A={a:+3d} B={b:+4d} deg  ->  {'도달 가능' if ok else '도달 불가'}   {lbl}")
