# P2 시간 파라미터화 — 사다리꼴 프로파일, 전역 단일 프로파일의 코너 가속도 버그 발견

- **Cycle**: 2026-08-31 11:10 KST
- **Branch**: `planning/p2-time-parameterize`
- **TODO**: `MP-0006` `time_parameterize` 사다리꼴 속도 프로파일
- **Phase**: P2
- **Status**: keep

## What I tried

STATE.md의 `Next claude-actionable` 1순위(MP-0006)를 선택했다. MP-0005(shortcut)는
Doing 상태였지만 PR #1이 이미 리뷰 대기 중이고 구현·테스트는 끝나 있어(재작업할
게 없음) 결정 트리 1번(진행 중 재개)을 실질적으로 적용할 대상이 없다고 판단,
2번 규칙(최우선 실행가능 Today)으로 넘어가 MP-0006을 골랐다. `research/feed.md`에
같은 날 올라온 researcher 노트(`research/2026-08/001.md`)가 이 TODO를 정확히
다뤄 설계 출발점으로 삼았다: "가장 오래 걸리는 관절이 세그먼트 시간을
결정한다"는 표준 다관절 동기화 규칙(moveit `IterativeParabolicTimeParameterization`
계열), TOPP-RA는 P5 후보로 보류.

처음엔 "웨이포인트마다 멈추면 RRT 원경로처럼 촘촘한 경로에서 부자연스럽다"고
판단해 전체 경로를 하나의 누적 호길이(Linf)로 재파라미터화하는 **전역 단일
사다리꼴 프로파일**을 구현했다. 단일 세그먼트 시험은 통과했지만, 다중
waypoint 합성 경로 속성 시험에서 실패했다.

## What worked / what failed

- **실패 → 원인 규명 → 재설계**: 전역 프로파일 시험에서 가속도 상한
  4.0 rad/s² 대비 실측 최대 173 rad/s²가 나왔다. 원인: 인접 두 세그먼트의
  방향이 다르면 그 경계에서 관절 속도의 *방향*이 순간적으로 바뀐다 — 속도
  *크기*는 각 세그먼트 안에서 항상 상한 이내(수학적으로 증명 가능, Linf 세그먼트
  길이 덕분)였지만, 방향 전환 자체가 사실상 무한 가속도다. RRT 원경로처럼
  코너가 촘촘할수록 이 문제가 커진다.
- **해결**: research 노트가 애초에 권장한 "세그먼트별 독립 사다리꼴 + 매
  waypoint에서 완전 정지" 방식으로 재구현. 이 저장소는 모든 관절이 같은
  스칼라 속도·가속도 상한을 쓰므로, 표준 다관절 동기화 규칙(각 관절의 필요
  시간 중 최댓값을 세그먼트 시간으로 채택)은 세그먼트 길이를 Linf(최대 성분)로
  재는 것과 수학적으로 동치임을 확인하고 그대로 구현에 반영했다.
- 코너에서 멈추지 않고 매끄럽게 잇는 blending(참고 자료 "Part 3")은 MP-0006
  범위 밖으로 명시적으로 보류 — 모듈 독스트링에 이유와 함께 기록해 향후
  "경로가 waypoint마다 멈칫거린다"는 피드백이 나오면 바로 참고할 수 있게 했다.
- `planning.trajectory.max_joint_speed_rad_s`는 새로 값을 정하지 않고
  `imitation.teleop.max_joint_speed_rad_s`(4.8, FFW-SH5 실기 관절 한계로 이미
  문서화된 값)를 그대로 재사용 — 두 설정이 어긋나 계획 결과가 하드웨어 한계를
  넘는 일을 방지하는 회귀 시험도 추가했다(`test_trajectory_speed_matches_hardware_joint_limit`).

## North-star delta

P2(경로 후처리)의 두 조각(shortcut, 시간 파라미터화) 모두 구현·테스트 완료
상태가 됐다(shortcut은 PR #1, 시간 파라미터화는 이번 PR #2 — 둘 다 리뷰 대기).
두 PR이 병합되면 P2가 완전히 닫히고, 다음은 P3(`planning.execution.follow_trajectory`,
`ArmTorqueController` 연결)이다.

## Key learnings

- **속도 상한을 지키는 것과 가속도 상한을 지키는 것은 다른 조건이다**:
  각 세그먼트 내부에서 속도 크기가 상한 이내라는 걸 증명해도, 세그먼트를
  이어붙일 때 방향이 바뀌면 가속도가 폭발할 수 있다. "웨이포인트에서 멈추는"
  기본형 사다리꼴이 정확도보다 매끄러움을 우선한 지름길이 아니라, 오히려
  이 불연속을 원천적으로 없애는 정확한 해법이었다.
- **모든 관절이 같은 스칼라 상한을 쓸 때, "관절별 독립 계산 후 최댓값으로
  동기화"와 "Linf 거리 하나로 재파라미터화"는 완전히 같은 결과를 낸다** —
  둘 중 구현이 더 간단한 쪽(Linf 거리)을 골라도 무방하다는 걸 수식으로
  확인했다. 나중에 관절별로 다른 속도 상한이 필요해지면 이 동치가 깨지므로
  그때는 진짜 관절별 계산으로 되돌아가야 한다.
- `research/feed.md`의 researcher 노트가 실제로 실행 가능한 설계 출발점을
  줬다 — Phase 0 인테이크가 값어치를 증명한 사례.

## Recommended next 1–3 priorities

1. MP-0013 벤치마크 하네스 — shortcut·time_parameterize 둘 다 "합성/비공식"
   수치만 있고 실제 can-sort 장면 정식 측정이 없다. 두 PR을 기다리는 대기열
   (MP-0004/0007/0014)이 늘어나고 있어 우선순위를 올릴 가치가 있다.
2. PR #1(MP-0005)·PR #2(MP-0006) 사람 리뷰/병합 — 병합되면 P2 완전히 닫힘.
3. P3(`planning/execution.py`, `ArmTorqueController` 연결, MP-0008) — 이번에
   나온 `Trajectory(times, positions)`를 실제로 소비하는 첫 지점.

## Artifacts

- PR: https://github.com/Geonhee-LEE/aiworker-mj-lab/pull/2
- Files touched: config/default.yaml, src/ffw_sh5_grasp/planning/trajectory.py (new), src/ffw_sh5_grasp/planning/__init__.py, src/ffw_sh5_grasp/planning/settings.py, tests/test_planning_trajectory.py (new), tests/test_planning_config.py, results/p2-time-parameterize.tsv
- TSV row appended: yes
