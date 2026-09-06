# Research State — auto-generated each cycle

_Last updated: 2026-09-06 · cycle p1-safety-certificate-profiling_

## North star distance

**P1 충돌 검사 가속 후보에 "지금은 하지 않는다"는 실측 근거 있는 결론이
났다.** `research/2026-09/005.md`가 조사해 둔 safety-certificate 스타일
캐싱(Bialkowski et al., IJRR 2016)을 바로 구현하지 않고 `scripts/
profile_certificate_caching.py`로 먼저 순이득 여부를 쟀다 — 실제 can-sort
장면에서 RRT-Connect가 방문한 configuration 표본에 대해
`ArmCollisionChecker.clearance()`(인증서 반경 후보)가 `is_valid()`보다
7.6~7.7배 비싸고, "인증서 반경이 같은 edge 위 이웃 waypoint를 전부 덮는다"는
낙관적 가정으로도 기대 절감(4.1~4.2회)이 그 비용비를 못 넘는다는 걸 장애물
포함/미포함 두 시나리오 모두에서 확인했다. 결론: **캐싱을 구현하지 않는다**
— 잘못된 최적화로 코드 복잡도만 늘리는 걸 막았다. PR #15 생성.

이 전에는 P4(Cartesian goal + 벤치마크 하네스)가 착수돼 `planning/goals.py`
(`solve_pose_goal`/`solve_pose_goal_multistart`)로 site pose 목표를
관절공간 `q_goal`로 변환하는 기능이 추가됐다(PR #14, 사람 리뷰 대기 —
아직 main에 없음). 그 전에는 MP-0007/MP-0017(RRT-Connect vs RRT* 50-seed
비교) 실측이 완료됐다 — 중앙값 6.82% 길이 감소, 통계적으로 유의한 경로 품질
차이 없음(Wilcoxon p≈0.86, PR #13, 사람 리뷰 대기).

## Current bottleneck

**PR 리뷰 큐** — #13(MP-0007/MP-0017 벤치마크 비교)·#14(MP-0011 Cartesian
pose goal IK)·#15(MP-0028 safety-certificate 프로파일링) 셋 다 사람 리뷰
대기. `MP-0012`(offline_pose_ik.py 위임)는 PR #14가 병합돼 `planning/
goals.py`가 main에 들어와야 착수 가능 — 그 전까지는 `MP-0018`/`MP-0019`
(P0 인프라)가 다음 자연스러운 착수 후보.

## Open experiments

| Branch | Last update | Last description | Days open |
|---|---|---|---|
| planning/p5-planner-comparison | 2026-09-06 | PR #13, MP-0007/MP-0017 실측 비교 — 사람 리뷰 대기 | 0 |
| planning/p4-cartesian-pose-goal-ik-seed | 2026-09-06 | PR #14, MP-0011 Cartesian pose goal IK 다중 재시도 — 사람 리뷰 대기 | 0 |
| planning/p1-safety-certificate-profiling | 2026-09-06 | PR #15, MP-0028 safety-certificate 캐싱 순이득 프로파일링(도입 보류) — 사람 리뷰 대기 | 0 |

## Recent learnings (last 3 cycles)

- **"바로 구현하지 않고 프로파일링부터"로 스코프를 좁힌 research TODO는
  "하지 않는다"는 결론도 유효한 성과다.** MP-0028: 프로파일링 스크립트와
  실측 데이터(TSV)를 남겨두면 장면/모델이 바뀌었을 때(장애물이 훨씬
  많아지는 등) 같은 도구로 재평가할 수 있다 — 결론을 코드에 하드코딩하지
  않고 재현 가능한 측정으로 남기는 게 핵심.
- **RRT-Connect가 실제로 방문하는 configuration 분포는 균등 무작위
  샘플링과 다르다.** 성능 프로파일링은 플래너가 실제로 호출하는 지점을
  써야 대표성이 있다 — `checker.is_valid`/`edge_checker.is_valid`를
  일시적으로 레코딩 래퍼로 바꿔치기(`try/finally`로 원상복구)하면 새 계측
  코드 없이 얻을 수 있다.
- **"test-only" 경고가 붙은 헬퍼도 실제로 프로덕션에서 재사용 가능한 로직을
  담고 있을 수 있다.** `tests/offline_pose_ik.py`는 "실시간 제품 API가
  아니다"라고 명시했지만, DLS/backtracking/다중 재시도 로직 자체는 올바른
  프로덕션 알고리즘이었다 — import 경계만 잘못돼 있었다. 다음 조각
  (`MP-0012`, PR #14 병합 후 착수)은 반대 방향으로 `offline_pose_ik.py`가
  `planning.goals`에 위임하도록 정리한다.
- **실행가능성 필터가 실제로 작동했다.** `MP-0012`가 STATE.md 1순위
  후보였지만 필요한 `planning/goals.py`가 미병합 PR #14 브랜치에만 있어
  main에서 분기 시 존재하지 않는다 — 브랜치 스택을 피하려고 건너뛰고
  2순위(`MP-0028`)로 넘어갔다.

## Next claude-actionable

1. `MP-0018`/`MP-0019`(P0) — `aggregate_results.py`는 코드가 이미 존재하고
   동작하는 것처럼 보이는데 TODO 상태가 Backlog로 남아 있다(불일치, 다음
   cycle에서 `git log`로 이미 구현됐는지 확인 후 Done 전환 또는 실제
   gap 파악). `todo_tool.py` 단위 테스트는 아직 미착수.
2. `MP-0012`(P4) — PR #14가 병합돼 `planning/goals.py`가 main에 들어오면
   바로 착수 가능(현재는 실행가능성 필터에 걸려 건너뜀).
3. `MP-0022`(P4) — aggregate_results.py에 Wilson CI 계산 추가(MP-0018 완료
   후가 더 자연스러움).

## Next user-blocked

1. **`MP-0030`** P7 Tier 2(결합형 whole-body 플래너) 착수 여부 결정 —
   타당성 평가 완료, 검증 시나리오(합성 MJCF)가 먼저 필요하다는 점까지
   문서화됨.
2. **`MP-0020`** Telegram 봇 생성 및 `telegram_setup.sh` 실행 (사람만
   가능).
3. **`MP-0021`**(hydrax)/`MP-0025`(VAMP-MR) 우선순위 결정 — 둘 다 조사는
   끝났고 구현 여부만 사용자 판단 대기.
4. **PR #13/#14/#15** 사람 리뷰/병합 대기.
5. **`MP-0014`**(pose goal 20 seed 성공률 측정) — `UserTest=☑`, 사람 확인
   필요.

## Cycles to date

19 (2026-08-30~09-06 사람 주도: P0 부트스트랩, P1 RRT-Connect 구현, 데모
반복/트리 시각화, 장애물 재배치, Q-space 시각화+CVD 팔레트, 인터랙티브 마우스
목표+버그 수정 3건, nullspace 정칙화+hydrax 조사, 데모 실행 경로에 shortcut+
시간 파라미터화 연결, RRT* 대안 플래너, CHOMP류 궤적 최적화 후처리, 벤치마크
하네스+P1 성공률 첫 측정, PR #1/#2 병합+P3 실행 모듈, PRD를 P7(모바일
매니퓰레이터)까지 확장 + P7.0 reachability map, P7.1 베이스 자세 선택,
P7 Tier 2 타당성 평가 + PR #3/#4/#5/#7/#8/#12 전체 병합(문서 중복 수동
정리 포함)로 PR 큐 완전 소진; 자율 루프: shortcut 평활화, 시간
파라미터화, MP-0007/MP-0017 벤치마크 비교, P4 Cartesian pose goal IK
다중 재시도, safety-certificate 캐싱 순이득 프로파일링(도입 보류))
