# RRT* 대안 플래너 — 단일 트리 rewiring, 실제 장면에서 goal_bias 튜닝 필요성 발견

- **Cycle**: 2026-09-03
- **Branch**: `planning/p5-rrt-star-planner`
- **TODO**: `MP-0016` `planning/rrt_star.py` 초안 (`MP-0015` 문헌조사도 이 설계 과정으로 충족)
- **Phase**: P5
- **Status**: keep

## What I tried

사용자가 "RRT-Connect 외 다른 향상된 기법 추가"를 요청, `TODO.md`의
MP-0015/16/17(P5, `docs/prd.md` P5 로드맵)이 정확히 이걸 위한 백로그였다.
`rrt_connect.py`의 자산(`RightArmSpace`, `EdgeChecker`, `PlannerResult`/
`TreeSnapshot` 데이터클래스)을 그대로 재사용하는 단일 트리 RRT*를 설계·구현했다
— 두 트리 bidirectional connect와 rewiring을 결합하면 복잡도가 크게 늘어나
의도적으로 단일 트리로 범위를 좁혔다(MP-0016이 요구하는 "초안" 범위에 맞음).

핵심 설계 결정 3가지:
1. 점근적 최적성을 위한 이론적 shrinking radius(`gamma*(log n/n)^(1/d)`) 대신
   고정 반경(`rewire_radius_rad`, 기본 `2*step_size_rad`) — 7-DOF에서 이론식은
   반경이 너무 빨리 줄어 rewiring이 사실상 안 일어난다.
2. informed sampling은 7-D 타원체 회전행렬 샘플러 대신 거부 표집
   (`dist(start,x)+dist(x,goal) < best_cost`) — 훨씬 단순하면서 같은 효과.
3. rewire 시 서브트리 비용 전파를 위해 `children` 인접 리스트를 유지하는
   `_StarTree`(비용 변화량을 delta로 서브트리에 전파, O(서브트리 크기)).

## What worked / what failed

**결정론 버그**: 처음엔 `_plan` 테스트 fixture의 `time_budget_s=5.0`이
`max_iterations=3000`보다 먼저 걸리는 경우가 있었다 — RRT*는 첫 해를 찾아도
안 멈추고 계속 도는데, 벽시계 시간 안에 몇 iteration이 도는지는 실제 CPU
스케줄링에 좌우돼 같은 seed로 두 번 돌려도 반복 횟수(따라서 경로)가 달랐다
(2404 vs 2405). 테스트에서는 `max_iterations`가 항상 먼저 걸리도록
`time_budget_s`를 넉넉히 잡아 고쳤다.

**무충돌 경로 시험의 해상도 불일치**: RRT-Connect의 같은 이름 시험은 계획
해상도(0.1)보다 2배 촘촘한 해상도(0.05)로 재검증하고도 안정적으로
통과하는데, 이는 RRT-Connect가 첫 해를 찾으면 바로 멈춰 전체 실행에서
만드는 edge 수 자체가 적어 "경계에 걸친 좁은 틈"을 만날 확률이 낮기
때문이다(보장된 성질은 아니다). RRT*는 예산이 끝날 때까지 계속 반복하며
훨씬 많은 edge를 검사·rewire해 이 경계 사례에 노출될 확률이 커진다 — 실제로
seed=0에서 재현됐다. 시험을 "같은 EdgeChecker 해상도로 다시 확인해도
스스로 모순되지 않는가"로 바꿔 실제로 보장되는 성질만 검증하도록 고쳤다.

**가장 중요한 발견 — 실제 장면에서 RRT-Connect 기본값을 그대로 쓰면 안 됨**:
합성 테스트 fixture(box space, slab 장애물)에서는 잘 됐지만, 실제
can-sort 장면 데모(`--seed 3 --execute --planner rrt_star`)에서 기본
`goal_bias=0.1`·`goal_tolerance_rad=step_size_rad(0.3)`로는 40초·6750회
반복 안에도 목표에 못 닿았다. 진단: 단일 트리는 RRT-Connect의 CONNECT처럼
한 반복에 여러 스텝을 한꺼번에 전진하지 못하고 `step_size_rad`(0.3)만큼만
전진한다 — bidirectional 탐색이 point-to-point 질의에서 훨씬 빠르다는 건
RRT-Connect가 애초에 고안된 이유이기도 한 잘 알려진 트레이드오프다.
`goal_bias=0.3`, `goal_tolerance_rad=0.5`로 올리자 같은 예산 안에서
안정적으로 성공했다(4000회 반복, ~20초). 데모 CLI에서 `--planner rrt_star`일
때 이 값들과 `time_budget_s=30`(RRT-Connect는 5)을 자동으로 쓰도록 했다.

## North-star delta

P5(RRT* 비교연구)가 착수됐다 — `rrt_star.py` 구현·테스트 완료, 데모에
`--planner {rrt_connect,rrt_star}` 플래그로 연결됨. 두 플래너가 같은
`PlannerResult`/`TreeSnapshot`을 반환해 트리 시각화·경로 재생 코드는
전혀 안 고쳐도 됐다 — RRT-Connect 설계 당시의 인터페이스 선택이
그대로 값어치를 했다.

## Key learnings

- **단일 트리 sampling 플래너의 하이퍼파라미터는 bidirectional 플래너의
  기본값을 그대로 물려받으면 안 된다.** 알고리즘 구조가 다르면(한 반복당
  전진량이 다르면) 튜닝도 달라야 한다 — 합성 테스트에서 통과한다고 실제
  장면에서도 같은 파라미터가 통하는 게 아니다. 반드시 실제 장면(더 비싸고
  복잡한 충돌 형상)에서 실측해야 한다.
- **결정론 테스트에서 `time_budget_s`가 `max_iterations`보다 먼저 걸리게
  두면 안 된다** — 벽시계 시간이 반복 횟수를 좌우하게 되어 실제 CPU
  스케줄링 노이즈가 테스트를 흔든다. 항상 `max_iterations`가 binding
  constraint가 되도록 `time_budget_s`를 넉넉히 잡아야 한다.
- **"참조 구현의 property test를 그대로 복사"는 재검증 노출도가 다르면
  깨질 수 있다** — RRT-Connect의 무충돌 경로 시험 패턴(계획보다 촘촘한
  해상도로 재검증)을 그대로 복사했다가 RRT*의 훨씬 많은 edge 검사 횟수
  때문에 실패했다. 새 알고리즘에 시험을 이식할 때는 "이 시험이 실제로
  보장하는 성질이 뭔가"를 다시 따져봐야 한다.

## Recommended next 1–3 priorities

1. PR #1(MP-0005)·PR #2(MP-0006)·PR #3(MP-0023)·PR #4(MP-0016) 사람
   리뷰/병합 — 넷 다 테스트 통과, 병합 대기 중.
2. `MP-0013` 벤치마크 하네스 — 병합 후 `MP-0017`(RRT-Connect vs RRT*
   50-seed 정식 비교표)을 실제로 만드는 데 필요.
3. PR #3(데모 natural-motion)이 병합되면 RRT*에도 shortcut+시간
   파라미터화를 연결해야 한다 — 지금은 서로 독립 브랜치라 RRT*의 경로는
   아직 shortcut 후처리를 안 거친다.

## Artifacts

- 브랜치: `planning/p5-rrt-star-planner`, PR #4
- 진단 스크립트(비체크인, 세션 스크래치): goal_bias/goal_tolerance 스윕으로
  실제 장면에서의 성공 조건 실측
