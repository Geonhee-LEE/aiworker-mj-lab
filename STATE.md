# Research State — auto-generated each cycle

_Last updated: 2026-09-06 · cycle p4-cartesian-pose-goal-ik-seed_

## North star distance

**P4(Cartesian goal + 벤치마크 하네스)가 착수됐다.** `planning/goals.py` 신규 —
`solve_pose_goal`(단일 시드 position-우선 DLS/backtracking)과
`solve_pose_goal_multistart`(순차 다중 재시도, 시드 실패 시 `RightArmSpace.sample()`
무작위 재시도 폴백)로 site pose 목표를 `plan_rrt_connect`/`plan_rrt_star`가
받는 관절공간 `q_goal`로 변환한다. 기존 `tests/offline_pose_ik.py`(test-only로
명시된 헬퍼)의 DLS 로직을 골자로 하되, 클리핑/샘플링은 `RightArmSpace`로
위임해 나머지 플래닝 모듈과 관례를 맞췄고 FK는 기존 `JointSpaceKinematics`를
그대로 재사용했다(새 FK 코드 없음). 실제 can-sort 장면(`full_scene.xml`, site
`grasp_target_r`)에서 근접 시드 단일 수렴, 원거리 시드 multistart 수렴, 도달
불가능한 목표에서 best-effort(미수렴이지만 관절범위 내 유지) 3개 신규 테스트,
전체 83개 통과. PR #14 생성.

이 전에는 MP-0007/MP-0017(shortcut 전/후 경로 길이·RRT-Connect vs RRT*
50-seed 비교) 실측이 같은 cycle 계열에서 완료됐다 — 중앙값 6.82% 길이 감소
(증가 0건), RRT-Connect·RRT* 사이 통계적으로 유의한 경로 품질 차이 없음
(Wilcoxon p≈0.86). PR #13으로 사람 리뷰 대기 중.

## Current bottleneck

**PR 리뷰 큐** — #13(MP-0007/MP-0017 벤치마크 비교)·#14(MP-0011 Cartesian
pose goal IK) 둘 다 사람 리뷰 대기. claude가 다음에 자연스럽게 집을 백로그는
남아 있다(`MP-0012` offline_pose_ik 위임, `MP-0028` safety-certificate 캐싱
조사, `MP-0018`/`MP-0019` 인프라).

## Open experiments

| Branch | Last update | Last description | Days open |
|---|---|---|---|
| planning/p5-planner-comparison | 2026-09-06 | PR #13, MP-0007/MP-0017 실측 비교 — 사람 리뷰 대기 | 0 |
| planning/p4-cartesian-pose-goal-ik-seed | 2026-09-06 | PR #14, MP-0011 Cartesian pose goal IK 다중 재시도 — 사람 리뷰 대기 | 0 |

## Recent learnings (last 3 cycles)

- **"test-only" 경고가 붙은 헬퍼도 실제로 프로덕션에서 재사용 가능한 로직을
  담고 있을 수 있다.** `tests/offline_pose_ik.py`는 "실시간 제품 API가
  아니다"라고 명시했지만, 그 DLS/backtracking/다중 재시도 로직 자체는
  올바른 프로덕션 알고리즘이었다 — import 경계(테스트 트리 vs `src/`)만
  잘못돼 있었다. `planning/goals.py`로 옮기며 클리핑/샘플링은
  `RightArmSpace`로 교체해 나머지 플래너와 규칙을 통일했다. 다음 조각
  (`MP-0012`)은 반대 방향으로 `offline_pose_ik.py`가 이 모듈에 위임하도록
  정리한다 — 지금은 두 구현이 병존해 논리가 살짝 중복돼 있다.
- **여러 PR이 같은 문서 섹션을 독립적으로 추가하면 `git`이 감지 못 하는
  "의미론적 중복" 충돌이 생긴다.** PR #3과 PR #4가 각자
  `docs/guide/motion-planning.md`에 거의 동일한 절을 추가했는데, 사이에
  다른 내용이 끼어 있어 git의 줄 단위 3-way merge가 충돌로 인식하지 못했다
  — 병합 후 파일을 실제로 읽어야만 발견된다.
- **`RightArmSpace.sample()`의 "unlimited면 0" 관례를 재사용할 때는 먼저
  대상 관절이 전부 `limited=True`인지 확인해야 한다.** 오른팔 7관절은
  전부 범위가 있어 문제가 되지 않았지만, P7 Tier 2(베이스 결합)처럼
  unlimited 자유도가 섞이면 조용한 버그가 될 수 있다는 점을 STATE.md
  이전 학습에서 미리 경고해 뒀던 것이 이번에 실제로 확인 절차로 이어졌다.
- **문서용 시각화라도 실측을 도는 비용이 생각보다 낮다.** P7.0/P7.1 HTML
  요약 아티팩트를 위해 실제로 `reachability.py`/`base_pose.py`를 실행해
  진짜 데이터를 얻었다 — 슬라이스로 쪼개고 restart 수를 줄이면 전체 격자
  재현 없이도 "진짜 데이터" 기준을 지킬 수 있었다(총 46초).

## Next claude-actionable

1. `MP-0012`(P4) — `tests/offline_pose_ik.py`를 `planning.goals`로 위임(중복
   제거) — `planning/goals.py`가 이미 있으니 바로 착수 가능.
2. `MP-0028`(P1) — safety-certificate 스타일 캐싱 도입 여부 판단(프로파일링
   부터, 바로 구현 금지).
3. `MP-0018`/`MP-0019`(P0) — 집계기 + todo_tool 단위 테스트, 인프라 정리.

## Next user-blocked

1. **`MP-0030`** P7 Tier 2(결합형 whole-body 플래너) 착수 여부 결정 —
   타당성 평가 완료, 검증 시나리오(합성 MJCF)가 먼저 필요하다는 점까지
   문서화됨.
2. **`MP-0020`** Telegram 봇 생성 및 `telegram_setup.sh` 실행 (사람만
   가능).
3. **`MP-0021`**(hydrax)/`MP-0025`(VAMP-MR) 우선순위 결정 — 둘 다 조사는
   끝났고 구현 여부만 사용자 판단 대기.
4. **PR #13/#14** 사람 리뷰/병합 대기.
5. **`MP-0014`**(pose goal 20 seed 성공률 측정) — `UserTest=☑`, 사람 확인
   필요.

## Cycles to date

18 (2026-08-30~09-06 사람 주도: P0 부트스트랩, P1 RRT-Connect 구현, 데모
반복/트리 시각화, 장애물 재배치, Q-space 시각화+CVD 팔레트, 인터랙티브 마우스
목표+버그 수정 3건, nullspace 정칙화+hydrax 조사, 데모 실행 경로에 shortcut+
시간 파라미터화 연결, RRT* 대안 플래너, CHOMP류 궤적 최적화 후처리, 벤치마크
하네스+P1 성공률 첫 측정, PR #1/#2 병합+P3 실행 모듈, PRD를 P7(모바일
매니퓰레이터)까지 확장 + P7.0 reachability map, P7.1 베이스 자세 선택,
P7 Tier 2 타당성 평가 + PR #3/#4/#5/#7/#8/#12 전체 병합(문서 중복 수동
정리 포함)로 PR 큐 완전 소진; 자율 루프: shortcut 평활화, 시간
파라미터화, MP-0007/MP-0017 벤치마크 비교, P4 Cartesian pose goal IK
다중 재시도)
