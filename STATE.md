# Research State — auto-generated each cycle

_Last updated: 2026-08-30 22:02 KST · cycle fix-marker-render-below-floor_

## North star distance

P0(관절공간 추상화 + 충돌 검사기)와 P1(RRT-Connect 코어)이 모두 구현·검증되었다.
P2(경로 후처리)는 shortcut 평활화(`planning.shortcut`, MP-0005)까지 구현·검증
완료. 실제 can-sort 장면에서 계획한 경로를 `ArmTorqueController`로 재생해
목표에 수렴하는 것까지 확인했다(데모 스크립트, 정식 P3는 아직 아님). 북극성까지는
P2 나머지(시간 파라미터화) → P3(정식 실행 모듈) → P4(Cartesian goal·벤치마크)
세 단계가 남았다.

## Current bottleneck

시간 파라미터화(`planning.trajectory`, MP-0006)가 없어 현재 경로는 waypoint를
곧바로 "수렴할 때까지 대기"하는 방식으로만 재생 가능하다(데모 스크립트의
임시방편). 정식 시간 파라미터화가 있어야 관절 속도 상한을 지키는 매끄러운
궤적을 만들 수 있다. 그 다음 병목은 벤치마크 하네스(MP-0013) 부재 — shortcut의
실제 효과(30% 목표 충족 여부)와 P1 성공률 모두 아직 `results/*.tsv`에 정식
기록이 없다(비공식 수치만 있음).

## Open experiments

| Branch | Last update | Last description | Days open |
|---|---|---|---|
| planning/p2-shortcut-smoothing | 2026-08-30 21:04 KST | MP-0005 shortcut 평활화, PR #1 리뷰 대기 | 0 |

## Recent learnings (last 3 cycles)

- **합성 시나리오의 shortcut 상한은 장면 형태에 달려 있다**: box-space 좁은 틈
  장면(30 seed)에서 median length reduction은 iterations=200에서 이미 23%로
  수렴하고 2000까지 늘려도 그대로였다 — 장애물 하나짜리 장면은 최적 우회
  경로 자체가 그 정도만 줄일 수 있는 형태였을 뿐, PRD의 30% 목표(실제
  can-sort 장면 기준) 충족 여부는 벤치마크 하네스 없이는 판단할 수 없다.
- **waypoint 기반 shortcut은 원본 경로 밀도에 의존한다**: RRT-Connect의
  `step_size_rad`가 작을수록 shortcut이 시도할 인덱스 쌍이 늘어 더 잘 줄어들
  것으로 예상되나 이번엔 검증하지 않았다 — 벤치마크할 때 함께 재보면 좋다.
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
- **`mocap_pos`를 쓰는 것만으로는 화면에 안 반영된다**: mocap body의 world
  pose(`data.xpos`, 렌더링에 실제 쓰이는 값)는 `mj_kinematics`/`mj_forward`가
  다시 돌아야 `mocap_pos`에서 재계산된다. 파이썬 코드가 최초로 mocap 위치를
  설정한 직후에는 명시적으로 `mj_forward`를 불러야 한다 — 안 그러면 컴파일
  시점 기본값(대개 월드 원점)에 남아 "바닥 밑에 있는 것처럼" 보인다.

## Next claude-actionable

1. **MP-0006** `time_parameterize` 사다리꼴 속도 프로파일 — P2를 마저 닫는다.
2. **MP-0013** `scripts/benchmark_planning.py` — TSV append, 2분 예산. shortcut
   전/후 길이·P1 성공률을 실제 can-sort 장면에서 재는 유일한 방법. MP-0004/
   0007/0014가 전부 이걸 기다리고 있어 우선순위를 올릴 가치가 있다.
3. **MP-0018** `aggregate_results.py`는 이미 있음 — 벤치마크 하네스(MP-0013)와
   연결해 P1/P2 결과를 `results/*.tsv`에 남기는 작업

## Next user-blocked

1. **MP-0020** Telegram 봇 생성 및 `telegram_setup.sh` 실행 (사람만 가능)
2. **MP-0004** can-sort 10 seed 성공률 정식 측정 — 벤치마크 하네스(MP-0013)가
   먼저 있어야 TSV로 기록 가능. 데모 스크립트로는 이미 여러 seed에서 성공을
   비공식 확인함(`scripts/demo_plan_right_arm.py --seed N --execute`)

## Cycles to date

6 (2026-08-30 사람 주도: P0 부트스트랩, P1 RRT-Connect 구현, 데모 반복/트리
시각화, 장애물 재배치, Q-space 시각화+CVD 팔레트, 인터랙티브 마우스 목표;
자율 루프: shortcut 평활화)
