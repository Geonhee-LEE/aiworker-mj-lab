# Research State — auto-generated each cycle

_Last updated: 2026-08-30 20:00 KST · cycle interactive-mouse-goal-marker_

## North star distance

P0(관절공간 추상화 + 충돌 검사기)와 P1(RRT-Connect 코어)이 모두 구현·검증되었다.
실제 can-sort 장면에서 계획한 경로를 `ArmTorqueController`로 재생해 목표에
수렴하는 것까지 확인했다(데모 스크립트, 정식 P3는 아직 아님). 북극성까지는
P2(평활화·시간화) → P3(정식 실행 모듈) → P4(Cartesian goal·벤치마크) 세 단계가
남았다.

## Current bottleneck

P2(shortcut 평활화 + 시간 파라미터화)가 없어 현재 경로는 waypoint를 곧바로
"수렴할 때까지 대기"하는 방식으로만 재생 가능하다(데모 스크립트의 임시방편).
정식 시간 파라미터화가 있어야 관절 속도 상한을 지키는 매끄러운 궤적을 만들 수
있다. MP-0005/MP-0006이 다음 최우선 후보다.

## Open experiments

| Branch | Last update | Last description | Days open |
|---|---|---|---|

## Recent learnings (last 3 cycles)

- **시작 상태 동기화를 두 번 빼먹음**: `--execute`와 `--interactive` 둘 다
  live `data.qpos`를 planner의 `start`로 먼저 맞추지 않아서, 여전히 상자와
  겹치는 `home` 키프레임 기준으로 동작해 실패했다. 새 실행 경로를 추가할
  때마다 "live 상태를 start와 동기화했는가"를 체크리스트로 챙길 것 —
  같은 실수를 두 번 했다.
- **안정성 감지와 "이미 처리함"은 다른 질문**: 인터랙티브 모드에서 "최근에
  안 움직였는가"만 보고 "마지막으로 처리한 위치와 같은가"를 구분 안 하면,
  마커가 가만히 있는 것 자체가 계속 "새로 안정됨"으로 재판정되어 무한
  재계획 루프가 된다. `poll_ref`(안정성)와 `processed_pos`(처리 완료)를
  분리해야 한다.
- **뷰어 세그폴트/느림**: `mujoco.viewer.launch_passive`는 수동 `.close()`
  대신 `with` 컨텍스트 매니저로만 쓰고, 결과 출력 뒤 `os._exit(0)`으로
  Python 정상 종료 절차를 건너뛰어야 일부 드라이버 조합(Wayland)에서
  세그폴트를 피한다. 매 물리 스텝(1kHz)마다 `viewer.sync()`하면 렌더
  오버헤드로 재생이 10배 이상 느려진다 — ~60Hz로 throttle해야 한다.
- **CVD 팔레트는 계산해야지 눈대중이면 안 된다**: 초록/주황처럼 "확실히
  달라 보이는" 조합도 protanopia 시뮬레이션에서 Delta E 2.8(문턱 6)로
  사실상 구분이 안 됐다. 이 환경엔 Node.js가 없어 `dataviz` 스킬의
  검증기 수식을 Python으로 포팅해 직접 돌렸다.
- **MuJoCo mocap body**는 뷰어 기본 조작(더블클릭+Ctrl드래그)만으로 커스텀
  마우스 코드 없이 드래그 가능한 3D 핸들을 만드는 표준 방법이다.

## Next claude-actionable

1. **MP-0005** shortcut 평활화(`planning/shortcut.py`)
2. **MP-0006** `time_parameterize` 사다리꼴 속도 프로파일
3. **MP-0018** `aggregate_results.py`는 이미 있음 — 벤치마크 하네스(MP-0013)와
   연결해 P1/P2 결과를 `results/*.tsv`에 남기는 작업

## Next user-blocked

1. **MP-0020** Telegram 봇 생성 및 `telegram_setup.sh` 실행 (사람만 가능)
2. **MP-0004** can-sort 10 seed 성공률 정식 측정 — 벤치마크 하네스(MP-0013)가
   먼저 있어야 TSV로 기록 가능. 데모 스크립트로는 이미 여러 seed에서 성공을
   비공식 확인함(`scripts/demo_plan_right_arm.py --seed N --execute`)

## Cycles to date

5 (2026-08-30 사람 주도: P0 부트스트랩, P1 RRT-Connect 구현, 데모 반복/트리
시각화, 장애물 재배치, Q-space 시각화+CVD 팔레트, 인터랙티브 마우스 목표)
