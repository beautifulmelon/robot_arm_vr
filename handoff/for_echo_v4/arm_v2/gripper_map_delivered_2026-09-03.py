"""그리퍼 서보각 -> URDF 관절값 변환.

퓨전에서 서보 조인트(Revolute 3)를 10~198도로 훑어 실측(95점)한 값을 피팅했다.
원본: export_data/gripper_sweep.json

평행4절이라 조는 회전 없이 반경 40.000 mm 원호로 평행이동한다
(전 범위 회전량 0.0000도, 진원 오차 0.0000 mm).

개구 0 ~ 82.52 mm.  품목별 필요 서보각:
    빨대 6mm->46도 / 페트병목 25->74 / 뚜껑 30->82 / 나무토막 40->94
    페트병몸통 65->134 / 알루미늄캔 66->136 / 종이컵 80->174

좌우가 대칭이 아니다. 구동로드가 서보 혼에 181.32도 차이로 붙어 있다(180 이어야 함).
그래서 계수를 좌우 따로 둔다.
"""
import numpy as np

SERVO_RANGE_DEG = (10.0, 198.0)
ROCKER_RADIUS_MM = 40.00
OPENING_RANGE_MM = (-0.30, 82.52)
ROCKER_L_ZERO_DEG, ROCKER_R_ZERO_DEG = 45.10, 52.47   # CAD 기준자세

# 7차, 최대오차 0.0536
ROCKER_L_COEF = [np.float64(-0.0), np.float64(1.11e-10), np.float64(-3.5025e-08), np.float64(5.324646e-06), np.float64(-0.000357027108), np.float64(0.003179846985), np.float64(0.062398542307), np.float64(59.679602620176)]
# 7차, 최대오차 0.0181
ROCKER_R_COEF = [np.float64(-0.0), np.float64(1.3e-11), np.float64(-3.216e-09), np.float64(2.89636e-07), np.float64(2.9852978e-05), np.float64(-0.010137085541), np.float64(0.4016650742), np.float64(56.105184826783)]
# 7차, 최대오차 0.0478
OPENING_COEF = [np.float64(0.0), np.float64(-8e-11), np.float64(2.6222e-08), np.float64(-4.238115e-06), np.float64(0.00029906215), np.float64(-0.000612770169), np.float64(-0.186621043829), np.float64(1.94502881233)]


def rocker_deg(servo_deg):
    """서보각(deg) -> (좌 로커각, 우 로커각) deg."""
    s = np.asarray(servo_deg, float)
    return np.polyval(ROCKER_L_COEF, s), np.polyval(ROCKER_R_COEF, s)


def joint_rad(servo_deg):
    """서보각(deg) -> (gripper_joint, rocker_r_joint) rad.  URDF 에 그대로 넣는 값."""
    l, r = rocker_deg(servo_deg)
    return np.radians(ROCKER_L_ZERO_DEG - l), np.radians(r - ROCKER_R_ZERO_DEG)


def opening_mm(servo_deg):
    """서보각(deg) -> 조 개구(mm)."""
    return np.polyval(OPENING_COEF, np.asarray(servo_deg, float))


def servo_for_opening(mm):
    """개구(mm) -> 서보각(deg). 단조구간 수치 역변환."""
    s = np.linspace(*SERVO_RANGE_DEG, 4001)
    return np.interp(np.asarray(mm, float), opening_mm(s), s)
