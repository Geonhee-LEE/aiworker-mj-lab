# Research State — auto-generated each cycle

_Last updated: 2026-08-31 07:17 KST · cycle nullspace-regularization-natural-posture_

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

- **nullspace 정칙화 없이는 여유 자유도가 매번 임의로 재배치된다**:
  position-only IK가 남기는 4개 자유도를 무작위 재시도에만 맡기면 팔
  자세가 매번 크게 바뀌어 "부자연스럽다"는 인상을 준다. 표준
  redundancy resolution(`dq = J⁺e + (I−J⁺J)(k(q_ref−q))`, 현재 관절값에
  가깝게 유지)을 추가했다. 단, "부자연스러워 보이는 해"가 실제로는 가장
  가까운 자세가 장애물과 충돌해서 정당하게 크게 재배치된 경우일 수 있다 —
  정칙화가 충돌 회피를 이기면 안 된다. P4(`planning.goals`) 설계에 반영.
- **IK 수렴 실패와 모션 플래닝은 다른 층의 문제**: IK는 "목표
  configuration이 존재하는가"를 풀고 모션 플래닝은 "존재하는 두
  configuration 사이 경로가 있는가"를 푼다. IK가 실패하면 애초에 계획할
  목표가 없으므로 플래닝이 개입할 수 없다. 불필요한 과잉 제약(안 물어본
  자세까지 고정)이 원인일 수 있다는 걸 항상 의심할 것.
- **`mocap_pos`를 쓰는 것만으로는 화면에 안 반영된다**: mocap body의 world
  pose(`data.xpos`, 렌더링에 실제 쓰이는 값)는 `mj_kinematics`/`mj_forward`가
  다시 돌아야 `mocap_pos`에서 재계산된다. 파이썬 코드가 최초로 mocap 위치를
  설정한 직후에는 명시적으로 `mj_forward`를 불러야 한다.
- **hydrax(github.com/vincekurtz/hydrax)는 GPU sampling MPC지 RRT류
  경로 탐색기가 아니다**: MPPI/CEM/DIAL-MPC 등으로 receding-horizon 최적
  제어를 푼다. 이 저장소에 들여온다면 RRT-Connect 대체가 아니라 아직 없는
  P3(정식 실행 모듈)에서 "전역 경로를 매끄럽게 따라가는 저수준 제어"로
  보완하는 역할이 맞다. 이 머신에 RTX 5080 + CUDA 12.8이 있어 기술적으로는
  가능하지만 JAX/MJX라는 무거운 새 의존성이 필요하고 hydrax README는 CUDA
  13을 명시한다 — 실제 설치 검증이 먼저다. 사용자 확인 대기, 아직 미착수.

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

7 (2026-08-30~31 사람 주도: P0 부트스트랩, P1 RRT-Connect 구현, 데모 반복/트리
시각화, 장애물 재배치, Q-space 시각화+CVD 팔레트, 인터랙티브 마우스 목표+버그
수정 3건, nullspace 정칙화+hydrax 조사; 자율 루프: shortcut 평활화)
