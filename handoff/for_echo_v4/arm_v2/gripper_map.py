"""그리퍼 서보각 -> URDF 관절값 변환.

퓨전에서 서보 조인트(Revolute 3)를 10~120도로 훑으며 실측한 값을 피팅했다.
생성: export_data/gripper_sweep.json  (56점)

평행4절이라 조는 회전 없이 반경 40.00 mm 원호를 따라 평행이동한다
(전 범위 회전량 0.0000도, 진원 오차 0.0000 mm).
그래서 URDF 는 조마다 revolute 2개(로커 + 역회전 mimic)로 정확히 표현된다.
prismatic 한 개로 근사하면 툴축 방향으로 5.46 mm 어긋난다.

좌우가 대칭이 아니다. 구동로드가 서보 혼에 +140.14도 / -41.18도 로 붙어
181.32도 차이 (180도여야 함). 그래서 계수를 좌우 따로 둔다.
"""
import numpy as np

SERVO_RANGE_DEG = (10.0, 120.0)
ROCKER_RADIUS_MM = 40.00

# 5차, 최대오차 0.0734
ROCKER_L_COEF = [np.float64(-5.3e-09), np.float64(1.5697e-06), np.float64(-0.0001107369), np.float64(-0.0051972233), np.float64(0.1970700829), np.float64(58.9077242729)]
# 5차, 최대오차 0.0198
ROCKER_R_COEF = [np.float64(-1e-10), np.float64(-6.14e-08), np.float64(5.035e-05), np.float64(-0.0107552421), np.float64(0.4104629724), np.float64(56.0600290035)]
# 5차, 최대오차 0.0356
OPENING_COEF = [np.float64(4.3e-09), np.float64(-1.4028e-06), np.float64(0.0001082302), np.float64(0.0060387859), np.float64(-0.2962475055), np.float64(2.5883045282)]


def rocker_deg(servo_deg):
    """서보각(deg) -> (좌 로커각, 우 로커각) deg. URDF joint value 로 쓴다."""
    s = np.asarray(servo_deg, float)
    return np.polyval(ROCKER_L_COEF, s), np.polyval(ROCKER_R_COEF, s)


def opening_mm(servo_deg):
    """서보각(deg) -> 조 개구(mm)."""
    return np.polyval(OPENING_COEF, np.asarray(servo_deg, float))


def servo_for_opening(mm):
    """원하는 개구(mm) -> 서보각(deg). 단조구간에서 수치 역변환."""
    s = np.linspace(*SERVO_RANGE_DEG, 2001)
    return np.interp(np.asarray(mm, float), opening_mm(s), s)
