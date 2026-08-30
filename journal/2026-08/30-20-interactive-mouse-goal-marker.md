# 마우스로 목표를 드래그하는 인터랙티브 모드 추가

- **Cycle**: 2026-08-30 20:00 KST
- **Branch**: (사람 주도, main 직접 작업)
- **TODO**: (신규 TODO 없음 — 사용자 직접 요청, `teleop_app.py`처럼 마우스로
  목표를 옮길 수 있는지 문의)
- **Phase**: P1 (데모 도구 개선) + P4 IK 조각의 조기 실험
- **Status**: keep

## What I tried

`teleop_app.py`는 GLFW 마우스 콜백을 직접 구현해 목표를 드래그하지만, 이
데모는 MuJoCo 뷰어에 이미 내장된 상호작용을 그대로 썼다 — ``mocap="true"``
body(``goal_marker``)를 씬에 추가하기만 하면 뷰어 기본 조작(더블클릭 선택 →
Ctrl+오른쪽 버튼 드래그)으로 ``data.mocap_pos``가 직접 갱신된다. 별도 마우스
이벤트 코드가 필요 없다.

이 위치를 폴링하다 안정되면 IK를 풀어야 하는데, 정식 IK 모듈(P4,
``planning.goals``)은 아직 없다. 저장소에 이미 있는
``tests/offline_pose_ik.py``(주석: "실시간 제품 API가 아니다")를 참고하되,
``tests/``를 ``scripts/``에서 직접 import하는 건 문서화된 모듈 경계를 넘는
것이라 데모 전용의 더 단순한 position-우선 DLS를 새로 짰다(``_ik_attempt``,
``_solve_valid_ik``) — 기존 ``JointSpaceKinematics``와 ``pose_error``는
정식 `src/` API라 그대로 재사용했다.

## What worked / what failed

**버그 1(재현)**: `--interactive`를 켜면 인터랙티브 루프에 들어가기 전에
live `data.qpos`를 `start`로 동기화하는 걸 빼먹었다 — `--execute` 경로에서
이미 한 번 겪은 것과 똑같은 버그를 새 코드 경로에 또 만들었다. 첫 자동
자기목표 재계획이 `home` 키프레임(상자와 겹치는 자세) 기준으로 IK를 풀어서
"IK는 풀렸지만 충돌 없는 해를 못 찾음"이 나왔다. `main()`의 인터랙티브
분기에도 `space.write(data.qpos, start)` + `mj_forward`를 추가해 고쳤다.

**버그 2(신규)**: 안정성 판정에 쓰는 "직전 폴링 틱과 같은 위치인가"만 보고
"마지막으로 실제 처리한 위치와 같은가"를 안 봐서, 사용자가 드래그를 멈춘
뒤에도 마커가 가만히 있는 것 자체가 계속 "새로 안정됨"으로 재판정되어
0.4초마다 같은 목표로 무한히 재계획을 반복했다. `poll_ref`(안정성 판정용)와
`processed_pos`(마지막 처리 위치)를 분리하고, 새 위치가 `processed_pos`에서
1cm 이상 떨어져 있을 때만 실제로 처리하도록 고쳤다.

두 버그 모두 실제 디스플레이(`DISPLAY=:0`)에서 백그라운드 스레드로
`data.mocap_pos`를 직접 써서 드래그를 흉내 내는 시험 스크립트로 재현·검증했다.
실제 `main(["--interactive"])` 경로로 재현했을 때는(임시 후킹 스크립트로
model/data/viewer 참조를 잡아서) 세그폴트 없이 exit 0으로 끝났다 — 직접
`_run_interactive`만 호출한 임시 테스트 하네스에서 세그폴트가 난 건
그 하네스 자체가 `os._exit(0)` 방어 코드 없이 Python 인터프리터 정상
종료 절차를 그대로 타서였다(이전 cycle에서 이미 겪고 고친 문제의 재확인).

## North-star delta

없음 — 데모 UX 개선.

## Key learnings

- "live 시뮬레이션 상태를 planner의 start와 동기화해야 한다"는 규칙을 새
  코드 경로를 만들 때마다 매번 의식적으로 챙겨야 한다 — 이번이 벌써 두 번째로
  똑같은 실수를 한 사례다. `--execute`, `--interactive` 둘 다 겪었으니, 다음에
  또 새로운 실행 경로를 추가할 때는 이 체크리스트 항목으로 미리 챙길 것.
- "안정 상태 감지"에서 "최근에 안 움직였다"와 "이전에 이미 처리했다"는
  서로 다른 질문이다 — 둘을 하나의 상태 변수로 합치면 조용히 무한루프가 된다.
- MuJoCo의 mocap body + 뷰어 기본 조작은 커스텀 마우스 피킹 코드 없이
  "드래그 가능한 3D 핸들"을 만드는 표준적이고 값싼 방법이다.

## Recommended next 1–3 priorities

1. MP-0005 shortcut 평활화
2. MP-0006 시간 파라미터화
3. (선택) 이번에 짠 position-우선 DLS IK를 `planning.goals`(P4, MP-0011)로
   정식 승격할지 검토 — 지금은 데모 전용 임시 버전

## Artifacts

- PR: 없음(사람 직접 작업)
- Files touched: scripts/demo_plan_right_arm.py, docs/guide/motion-planning.md
- TSV row appended: no
