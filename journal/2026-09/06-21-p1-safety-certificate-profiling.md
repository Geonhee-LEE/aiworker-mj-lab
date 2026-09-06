# P1 safety-certificate 캐싱 순이득 프로파일링 — 도입 보류

- **Cycle**: 2026-09-06 21:03 KST
- **Branch**: `planning/p1-safety-certificate-profiling`
- **TODO**: `MP-0028` [research] EdgeChecker/ArmCollisionChecker에 safety-certificate 스타일 캐싱 도입 여부 판단
- **Phase**: P1
- **Status**: keep

## What I tried

`research/2026-09/005.md`가 조사해 둔 아이디어(Bialkowski et al., IJRR 2016
safety certificates)를 바로 구현하지 않고, 먼저 이 저장소에서 순이득이 나는지
실측했다. `scripts/profile_certificate_caching.py`를 새로 작성해:

1. 실제 can-sort 장면에서 RRT-Connect를 5~15 seed 돌려, 트리 확장·edge 이분
   검사 과정에서 `is_valid()`에 실제로 넘겨진 configuration 표본을 수집(균등
   무작위가 아니라 플래너가 실제로 방문하는 분포를 그대로 씀).
2. 그 표본에서 최대 200개를 뽑아 `is_valid()`/`clearance()` 각각 호출당
   중앙값 시간(반복 20회로 노이즈 완화)을 측정.
3. 같은 표본의 `clearance()` 값(인증서 반경 후보) 분포를 `resolution_rad`
   (저장소 관례값 0.05, `benchmark_planning.py`와 동일)와 비교.
4. "인증서 반경이 같은 edge 위 이웃 waypoint를 전부 덮는다"는 낙관적 가정
   아래 기대 절감 호출 수(`2 * median_clearance / resolution_rad`)를
   `clearance/is_valid` 비용비와 비교해 손익분기 판정.

장애물 포함(빡빡한 clearance)·미포함(여유 clearance) 두 시나리오 모두 실행.

## What worked / what failed

- 두 시나리오 모두 일관된 결론: `clearance()`가 `is_valid()`보다 약
  7.6~7.7배 비싸고, 낙관적 기대 절감(4.1~4.2회)이 그 비용비를 넘지 못한다
  → **safety-certificate 캐싱을 지금 구현하지 않는다.**
- 전체 실행이 seed당 ~0.4초로 매우 빨라(15 seed 기준 5.9초), 2분 예산에
  여유가 크다.
- 사소한 관찰: 두 시나리오에서 p10/median clearance 값이 거의 동일하게
  나옴 — `collision_distance_gradient`가 `max_distance`(=`clearance_report_m`,
  기본 0.2) 밖의 pair는 제외하고 남은 pair 중 최솟값을 취하는데, 방문한
  대부분의 표본에서 지배 pair가 `table_top` 모드(팔-테이블 높이 기반 거리)라
  변동폭이 작았을 가능성이 있다 — 추가 조사 없이 결론에는 영향 없음(둘 다
  이미 손익분기 미달).

## North-star delta

북극성(충돌 없는 경로 계획·실행) 자체는 바뀌지 않음 — 이번 cycle은 P1 충돌
검사기의 성능 최적화 여부를 "구현 전에 재고 판단"하는 research TODO였고,
결론은 "지금 이 구조에서는 하지 않는다"다. 잘못된 최적화로 코드 복잡도만
늘리는 것을 막았다는 점에서 북극성에 간접 기여(불필요한 작업 회피).

## Key learnings

- **"바로 구현하지 않고 프로파일링부터"로 스코프를 좁힌 TODO는 실제로
  "하지 않는다"는 결론이 나올 수 있고, 그것도 유효한 성과다.** 프로파일링
  코드(재사용 가능한 스크립트)와 실측 데이터는 TSV에 남아 향후 장면이
  바뀌거나(예: 장애물이 훨씬 많아지는 경우) 재평가할 근거가 된다.
- **RRT-Connect가 실제로 방문하는 configuration 분포는 균등 무작위 샘플링과
  다르다** — `checker.is_valid`/`edge_checker.is_valid`를 일시적으로
  레코딩 래퍼로 바꿔치기해 "진짜" 프로파일링 표본을 얻는 패턴이 유용했다
  (`try/finally`로 원상복구).
- 이 프로파일링 방법론 자체가 재사용 가능 자산이다 — 장면/모델이 바뀌어
  `clearance()` 비용비가 낮아지거나 인증서 반경이 커지면 같은 스크립트로
  재평가하면 된다.

## Recommended next 1–3 priorities

1. `MP-0018`/`MP-0019`(P0) — `aggregate_results.py`는 이미 구현돼 있는 것처럼
   보이는데 TODO가 Backlog로 남아 있다(불일치, 다음 cycle에서 확인 후 Done
   전환 검토). `todo_tool.py` 단위 테스트는 여전히 미착수.
2. PR #13/#14/#15 사람 리뷰/병합 대기(리뷰 큐가 계속 병목).
3. `MP-0012`는 아직 실행 불가 — 필요한 `planning/goals.py`가 main이 아니라
   미병합 PR #14 브랜치에만 있음. PR #14가 병합되면 다음 cycle에서 바로
   착수 가능.

## Artifacts

- PR: https://github.com/Geonhee-LEE/aiworker-mj-lab/pull/15
- Files touched: `scripts/profile_certificate_caching.py`,
  `tests/test_planning_certificate_profiling.py`,
  `results/p1-safety-certificate-profiling.tsv`
- TSV row appended: yes
