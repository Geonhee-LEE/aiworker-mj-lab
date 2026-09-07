# MP-0031: 장애물 배치 다양화 — RRT-Connect vs RRT* 결론이 뒤집힘

- **Cycle**: 2026-09-06
- **Branch**: `planning/p5-planner-comparison` (PR #13, 추가 커밋)
- **TODO**: MP-0031
- **Phase**: P5
- **Status**: keep

## What I tried

사용자가 MP-0007/MP-0017 실측 결과(RRT-Connect vs RRT* 비교, PR #13)에
대해 "seed·장애물 배치에 따라 성능이 달라질 것 같아 벤치마킹이 의미
있는지 우려된다"고 지적 — seed 변동성은 이미 대응 표본 검정으로
통제돼 있었지만, **장애물 배치 자체는 정말로 통제 안 돼 있었다**
(고정 3-구체 배치의 on/off 스위치뿐, 배치를 바꿀 방법이 없었음).

1. `demo_plan_right_arm.py`의 `_build_scene()`에 `spheres=` 오버라이드를
   추가(기본값은 기존 `OBSTACLE_SPHERES`라 완전히 하위호환), 장애물
   관련 상수를 `BASE_REQUIRE_CONTACT_GEOMS` + 임의 개수 장애물 이름으로
   재구성.
2. `benchmark_planning.py`에 `--obstacle-layout {default,narrow_passage,cluttered}`
   추가. 새 배치 2개는 직접 실측으로 검증한 뒤 채택했다 — `DEFAULT_START`가
   유효해야 하고, RRT-Connect 반복 횟수 분포가 "default"보다 뚜렷이
   넓게 퍼져야(=진짜로 더 어려워야) 한다는 두 기준. 첫 시도(narrow_passage
   v1)는 팔의 홈 자세와 겹쳐 무효였다 — 위치를 몇 번 조정해 재검증했다.
3. 같은 50 seed로 6번 실측(장애물 없음/narrow_passage/cluttered ×
   RRT-Connect/RRT*).

## What worked / what failed

**결론이 실제로 뒤집혔다.** 장애물 없는 시나리오에서는 "차이 없음"
(Wilcoxon p=0.86)이었지만, 두 개의 진짜 어려운 배치에서는:
- RRT-Connect: 두 배치 모두 50/50 성공.
- RRT*: narrow_passage 44/50(88%), cluttered 39/50(78%) — **RRT-Connect
  보다 확실히 덜 안정적**(15초 예산 안에 첫 해도 못 찾는 경우가 생김).
- 그런데 둘 다 성공한 seed들만 놓고 보면 RRT*가 **통계적으로 유의하게
  더 짧은 경로**를 찾는다(narrow_passage p=0.0037, cluttered p=0.0198).

즉 RRT*는 "쓸모없다"가 아니라 "안정성을 경로 품질과 맞바꾼다"는 게
진짜 그림이었다 — 사용자의 우려가 정확했다.

**작업 중 실수 하나 발견·수정**: 워킹 디렉토리에서 자율 cron 루프가
동시에 `research/cron_activity.md`를 계속 갱신하고 있었는데, `git
stash pop`이 main에 이미 반영된 cron 커밋과 충돌했다. 그 충돌을
`git add`로 그냥 넘기고 `state_push.sh`로 커밋해버려 **literal git
conflict marker(`<<<<<<<`/`=======`/`>>>>>>>`)가 main에 그대로 push된
사고**가 있었다(커밋 `f22721a`). 발견 즉시 두 로그 섹션을 시간순으로
수동 병합해 marker를 제거하고 정정 커밋(`e678d9e`)을 push했다 — 내용
손실은 없었지만, `git add` 전에 파일을 실제로 열어 확인하지 않은 게
원인이다.

## North-star delta

RRT-Connect vs RRT* 비교가 이제 "장애물 없을 때 차이 없다"는 좁은
관찰에서 "장애물 유무·배치에 따라 트레이드오프의 성격 자체가 달라진다"
는 훨씬 견고한 결론으로 바뀌었다. `--obstacle-layout`이라는 재사용
가능한 벤치마크 축도 하나 더 생겼다.

## Key learnings

- **사용자가 벤치마크의 통계적 타당성을 직접 지적했을 때, 어느 축이
  이미 통제됐고 어느 축이 안 됐는지 코드로 정확히 확인하고 답해야
  한다** — "seed는 이미 paired test로 통제했다"와 "장애물 배치는
  전혀 통제 안 됐다"를 구분해서 설명한 게 다음 작업 범위를 명확하게
  했다.
- **새 장애물 배치를 코드에 박아 넣기 전에 반드시 실측으로 검증해야
  한다** — "그럴듯해 보이는 좌표"가 실제로는 팔의 시작 자세와 겹쳐
  무효였다(1차 시도). 이 저장소의 기존 습관(원래 3-구체 배치도 5000개
  표본으로 실측 튜닝됨)을 그대로 따랐다.
- **conflict marker가 남은 채로 `git add`하면 안 된다** — 자동화 스크립트
  (`state_push.sh`)는 파일 *내용*이 실제로 유효한지는 검증하지 않는다
  (화이트리스트 경로 검사만 한다). 병합 충돌이 났을 땐 파일을 반드시
  다시 읽고 marker가 없는지 확인한 뒤 스테이징해야 한다 — 이번엔 운
  좋게 내용 손실 없이 복구했지만, 다음엔 이 확인을 습관화해야 한다.

## Recommended next 1–3 priorities

1. PR #13 사람 리뷰/병합 — 이제 6개 시나리오 실측이 다 들어있다.
2. 사용자가 이 트레이드오프(RRT*=품질 vs 안정성/속도)를
   `docs/guide/motion-planning.md`에 반영할지, 15초 예산을 늘려
   재검증할지 결정하면 그에 따라 진행.
3. PR #14(MP-0011)/#15(MP-0028, 도입 보류 판정) 사람 리뷰/병합.

## Artifacts

- PR: https://github.com/Geonhee-LEE/aiworker-mj-lab/pull/13 (갱신됨)
- 실측 데이터: `results/p5-obstacle-layouts.tsv`(300행)
- 정정 커밋: `main` `e678d9e`(cron_activity.md conflict marker 제거)
