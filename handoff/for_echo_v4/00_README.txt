================================================================================
  신규 5축 팔 v2 + 그리퍼 + VR 조종 화면  —  echo_v4 인계 꾸러미
================================================================================
  작성일 : 2026-09-01
  보낸이 : VR ↔ Isaac Sim 연동 담당 (Mac 시뮬 담당)
  받는이 : echo_v4 / third_party/robot_arm_vr 담당
  대상   : dingdongdengdong/echo_v4  →  third_party/robot_arm_vr
================================================================================


 0. 30초 요약
--------------------------------------------------------------------------------
   신규 5축 팔(Damiao DM-J4340P ×5) + 평행 그리퍼(Feetech STS3215) 를
   Quest 2 로 조종할 수 있게 등록·검증한 결과 전부입니다.

       팔 URDF        arm_v2/          (기구 담당 제공 + 제가 만든 IK 전용판)
       텔레옵 설정     config/arm_v2.json   ← 홈 자세 포함
       새 파일        new_files/       그대로 복사하면 됩니다 (충돌 없음)
       기존 파일 변경  patches/         3-way 로 적용하세요 (§4)
       설명           01~04 txt

   ★ 그리퍼가 VR 오른손 트리거로 움직입니다. Mac 송신부는 열려 있고
     **받는 쪽(젯슨/브릿지)만 grasp → 서보각 한 줄을 붙이면 됩니다.**


 1. 읽는 순서
--------------------------------------------------------------------------------
   00_README.txt        이 문서. 전체 지도
   01_파일설명.txt       ★ 파일 하나하나가 무엇인지. 제일 자세합니다
   02_조종방법.txt       사이트 구성 + VR 조종법 + 실행 명령
   03_홈자세.txt         첫 포즈를 어떻게 정했나. 숫자와 근거
   04_적용방법.txt       echo_v4 에 실제로 넣는 절차

   docs/67_기구담당_원본_v2.txt   ★ URDF 를 만든 사람이 쓴 원본. §7 함정 필독
   docs/68_젯슨_인수인계.txt      실물 조종 담당용
   docs/ARM_VERSIONS.md          버전 대장 (v1/v2 차이)
   docs/07_trash_task_roadmap.md 쓰레기 수거 태스크 측정 리스트·로드맵


 2. 폴더 구조
--------------------------------------------------------------------------------
   arm_v2/                       팔 자산. assets/arm_v2/ 로 넣습니다
       arm_v2.urdf                 원본 (그리퍼 mimic)
       arm_v2_nomimic.urdf         좌우 로커 독립
       arm_v2_ik.urdf              ★ 텔레옵이 실제로 쓰는 것
       meshes/  10개               mm 단위 STL
       renders/ 13장               CAD·URDF·카메라 시점
       gripper_map.py              서보각 ↔ 로커각 ↔ 개구 변환 (실측 피팅)
       gripper_sweep.json          서보 스윕 원본 56점
       _props.json                 질량/COM/관성 원본

   config/arm_v2.json            텔레옵 설정 + 홈 자세

   new_files/                    없던 파일. 그대로 복사
       src/rpo_teleop/gripper_link.py
       scripts/10_camera_preview.py 11_make_ik_urdf.py 12_arm_preview.py
               13_pose_snapshot.py 14_reach_map.py
       web/vendor/arms.json

   patches/                      기존 파일 변경분
       00_all.patch                아래 3개를 합친 것
       01_teleop_grasp.patch       ★ 그리퍼 송신 (제일 중요)
       02_xr_server_nocache.patch  헤드셋 캐시 문제
       03_web.patch                VR 실제 형상 + 대시보드 팔 선택기

   docs/                         참고 문서


 3. ★ 주의 — 우리 저장소가 갈라져 있습니다
--------------------------------------------------------------------------------
   beautifulmelon/robot_arm_vr 와 dingdongdengdong/robot_arm_vr 는
   같은 뿌리(6e5745b)에서 갈라져 각자 갔습니다.

       그쪽에만 있음   SuperArm J1J2 텔레옵 · Quest 홈버튼 · J1J2 URDF (3커밋)
       이쪽에만 있음   신규 5축 팔 v1/v2 · 그리퍼 조종 (2커밋)

   ★ 그래서 **파일을 통째로 덮어쓰지 마세요.** 특히 scripts/05_teleop_sim.py 는
     그쪽에서 +117 줄을 고쳤습니다. patches/ 를 쓰는 이유가 이것입니다.

   patches 는 **공통 조상 6e5745b 기준**으로 뽑았습니다. `git apply -3` 로
   3-way 적용하면 그쪽 변경과 자동으로 합쳐집니다. 자세한 건 04 문서에.


 4. 검증 상태
--------------------------------------------------------------------------------
   테스트 131개 통과
   config/arm_v2.json 으로 기동 → robot=arm_v2_ik.urdf, 링크 10개,
       dof_mismatch None, STL 200
   가짜 젯슨(07_fake_jetson.py) + 가짜 Mac(08_fake_mac.py) 으로
       HOLD→RUN, 관절 추종, 손실 0, 트립 0 확인

   ★ 아직 안 한 것
       · 실물 서보 구동 (드라이버·영점·부호 — 젯슨 담당 영역)
       · 그리퍼 파지력 측정
       · 본체 카메라 기울기 확정

================================================================================
