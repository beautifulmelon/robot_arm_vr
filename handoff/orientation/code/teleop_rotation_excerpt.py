# 05_teleop_sim.py 에서 방향과 관련된 부분만 발췌 (원본은 그쪽 레포에도 있음)

# ── 손목 롤 축 실측 ──────────────────────────────────────────────────
def measure_wrist_axis(ik: "ArmIK", cfg: ArmConfig) -> tuple[np.ndarray, float, float]:
    """끝관절을 조금 돌려 **EE 로컬 회전축**을 실측한다.

    ★ 축을 코드에 박으면 안 된다. 팔마다 다르고, 심지어 **부호가 반대**다.

        config/arm.json       joint5  EE로컬 축 = [0, -1, 0]
        config/arm_temp.json  joint3  EE로컬 축 = [0, +1, 0]   ← 부호 반대

      예전 구현은 X축 [w, 0, 0] 을 썼는데 실제 축은 둘 다 Y 였다. 그래서 팔이
      **낼 수 없는 회전을 요청**하고 있었다. 자세는 soft task(가중치 1 : 위치
      10000)라 예외 없이 "갈 수 있는 가장 가까운 자세"로 수렴하는데, 결과가
      엉망이었다 (실측):

          전체 팔  : 자세오차 최대 117.7°, 음의 방향이 사실상 죽음
          임시 팔  : 끝관절이 **아예 안 움직임**, 오차 177.7°

      URDF 가 진실이므로 기동할 때 한 번 재서 쓴다.

    Returns:
        (축 단위벡터, 롤 하한 rad, 롤 상한 rad)
        하한/상한은 **홈 자세 기준 상대값**이다. 끝관절이 홈에서 이미 얼마쯤
        돌아가 있으면 양쪽으로 갈 수 있는 양이 다르다. 대칭으로 잡으면 한쪽에
        데드존이 생긴다.
    """
    q0 = np.asarray(cfg.home, dtype=float)
    ik.set_q(q0)
    T0 = ik.fk()
    q1 = q0.copy()
    q1[-1] += np.radians(5.0)
    ik.set_q(q1)
    T1 = ik.fk()
    ik.set_q(q0)

    R = T0[:3, :3].T @ T1[:3, :3]
    ang = float(np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)))
    if ang < np.radians(0.5):
        # 끝관절이 EE 자세를 거의 안 바꾸는 팔. 손목 롤을 줄 수 없다.
        return np.zeros(3), 0.0, 0.0
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(ang))
    axis = axis / np.linalg.norm(axis)

    lo = float(cfg.lower[-1] - q0[-1])
    hi = float(cfg.upper[-1] - q0[-1])
    return axis, lo, hi


# ── 목표 자세 합성 (회전 순서) ───────────────────────────────────────
        # 목표 EE pose = 홈 자세 기준 + 클러치 변위/회전 + 손목 롤
        target = np.eye(4)
        target[:3, 3] = home_ee[:3, 3] + delta_pos
        # ★ 회전 적용 순서 주의
        #   클러치가 주는 rel_rotvec 은 **월드 기준** 상대 회전이다 (R_now · R_ref⁻¹).
        #   따라서 왼쪽에서 곱해야 한다. 오른쪽에 곱하면 바디 기준으로 해석되어
        #   회전축이 home 자세만큼 틀어진다 (실측: 축이 170.4° 어긋나 손목을 돌리면
        #   로봇 손이 거의 반대로 돈다).
        #   반면 손목 롤(썸스틱)은 EE 자기 축 기준이라 오른쪽에 곱하는 게 맞다.
        # ★ 축은 measure_wrist_axis() 가 URDF 에서 실측한 것이다. 박으면 안 된다
        #   — 팔마다 다르고 부호까지 반대다 (그 함수 주석 참고).
        roll = rotvec_to_rotation(roll_axis * wrist_roll)
        target[:3, :3] = rotvec_to_rotation(rel_rotvec) @ home_ee[:3, :3] @ roll

        t_ik = time.perf_counter()

# ── 화면 뷰 변환 = 제어 매핑의 역행렬 ─────────────────────────────
        # 브라우저(Quest HUD / 대시보드)로 로봇 상태 발행.
        # 헤드셋을 쓰면 Mac 화면을 못 보므로 관절 한계를 VR 안에서 봐야 한다.
        ee_now = ik.fk()
        reach_now = float(np.linalg.norm(ee_now[:3, 3] - cfg.shoulder))
        # 뷰 변환 = 제어 매핑의 역행렬. 이렇게 해야 "손을 오른쪽으로 옮기면
        # 화면에서도 EE 가 오른쪽으로" 가 미러/yaw 설정과 무관하게 항상 성립한다.
        # ★ 링크마다 적용하지 않고 로봇 전체 그룹에 한 번만 적용한다 (arm_visual.poses 참고).
        view = np.linalg.inv(clutch.mapping)
        state = build_joint_state(cfg, q, clamped=clamped)
        state.update({
            "view_matrix": view_matrix4(view),
            "geometry": visual.geometry,
            "links": visual.poses(),
            "ee_point": visual.point(ee_now[:3, 3]),
            "target_point": visual.point(target[:3, 3]),
            "workspace": {"origin": visual.point(cfg.shoulder),
                          "min": cfg.min_reach, "max": cfg.max_reach},
