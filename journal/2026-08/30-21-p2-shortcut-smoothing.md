# RRT-Connect 경로 shortcut 평활화 (P2 첫 조각)

- **Cycle**: 2026-08-30 21:04 KST
- **Branch**: `planning/p2-shortcut-smoothing`
- **TODO**: `MP-0005` [planner] shortcut 평활화 `planning/shortcut.py`
- **Phase**: P2
- **Status**: keep

## What I tried

`planning.shortcut` 모듈을 새로 추가: `shortcut_path(space, edge_checker, path,
*, rng, iterations)`가 경로에서 무작위로 두 waypoint를 골라 그 사이 직선
구간이 `EdgeChecker`로 무충돌이면 그 사이를 통째로 잘라낸다. 이미 planner가
검증한 원래 waypoint끼리 잇는 것이므로 끝점 재검사(`check_endpoints=False`)는
생략해 낭비를 줄였다. `path_length_rad(space, path)` 헬퍼도 함께 추가(벤치마크
MP-0007/0013에서 재사용 목적). 둘 다 `planning/__init__.py`에 재수출.

`tests/test_planning_shortcut.py`에 6개 순수 numpy 속성 시험을 추가:
길이 계산 정확성(2개), 자유공간 지그재그가 직선 근처로 줄어드는지, 실제
RRT-Connect 출력(좁은 틈 시나리오, 10 seed)에 적용했을 때 길이 비증가·
무충돌·끝점 보존이 모두 성립하는지, 결정론, 2점 이하 경로는 그대로 두는지.

## What worked / what failed

한 번에 구현·시험 모두 통과했다 — 별도로 실패한 접근은 없었다. `EdgeChecker`가
이미 이분(bisection) 순서로 구간을 검사하는 API를 제공해 shortcut 쪽에서
따로 충돌 검사 로직을 새로 짤 필요가 없었다(R-F-003/P0 산출물을 그대로
재사용). waypoint 인덱스 기반 shortcut이라 RRT-Connect가 만드는 트리 경로
구조와 자연스럽게 맞아떨어졌다.

## North-star delta

R-F-004(경로 후처리) 중 shortcut 절반이 끝났다. `planning.trajectory`(시간
파라미터화, MP-0006)가 남아야 P2 전체가 닫힌다. P2 exit criterion("경로 길이
중앙값 30%+ 단축")은 이 TODO 단독으로 주장하지 않았다 — 아래 참고 수치는
비공식이다.

## Key learnings

- **합성 slab 시나리오의 shortcut 상한은 시나리오 형태에 달려 있다**: box-space
  좁은 틈 장면(30 seed)에서 median length reduction은 iterations=200에서
  이미 23%로 수렴하고 2000까지 늘려도 그대로였다 — 이 장애물 하나짜리 장면은
  최적 우회 경로 자체가 직선 대비 그 정도만 줄일 수 있는 형태였기 때문으로
  보인다. PRD의 30% 목표는 실제 can-sort 장면 기준이라 이 수치로 미달/충족을
  판단할 수 없다 — 벤치마크 하네스(MP-0013)가 있어야 진짜 숫자가 나온다.
- **waypoint 기반 shortcut은 원본 경로 밀도에 의존한다**: RRT-Connect가 촘촘한
  트리를 만들면(작은 `step_size_rad`) shortcut이 시도할 인덱스 쌍이 많아져
  더 잘 줄어들 것으로 예상된다. 이번엔 검증하지 않았다 — 다음에 벤치마크할 때
  `step_size_rad` 대비 shortcut 효과를 같이 재보면 좋겠다.

## Recommended next 1–3 priorities

1. MP-0006 `time_parameterize` 사다리꼴 속도 프로파일 — P2를 마저 닫는다.
2. MP-0013 `scripts/benchmark_planning.py` — shortcut 전/후 길이를 실제
   can-sort 장면에서 재는 유일한 방법. MP-0004/0007/0014가 전부 이걸 기다리고
   있어 우선순위를 올릴 가치가 있다.
3. MP-0007 shortcut 전/후 경로 길이 비교 속성 시험(벤치마크 하네스 이후,
   실제 장면 데이터로).

## Artifacts

- PR: https://github.com/Geonhee-LEE/aiworker-mj-lab/pull/1
- Files touched: `src/ffw_sh5_grasp/planning/shortcut.py` (new),
  `src/ffw_sh5_grasp/planning/__init__.py`, `tests/test_planning_shortcut.py` (new),
  `results/p2-shortcut-smoothing.tsv` (new)
- TSV row appended: yes
