# P7.0/P7.1 가이드 문서화 + 실측 데이터 기반 HTML 요약

- **Cycle**: 2026-09-05
- **Branch**: `docs/p7-mobile-manipulator-guide`
- **TODO**: `MP-0029`
- **Phase**: P7
- **Status**: keep

## What I tried

사용자가 "[P7.0/P7.1] 각각에 대해서 docs 업데이트 및 하나의 html로 각각
컨텐츠를 볼 수 있도록 구성해주세요"라고 요청 — PR #10(reachability map)·
#11(base_pose + mobile_execution)이 병합됐지만 `docs/guide/motion-planning.md`
에는 아직 반영이 안 돼 있었다.

1. `docs/guide/motion-planning.md`에 "모바일 매니퓰레이터 P7" 절을 신규
   추가(PR #12) — 기존 P0/P1/P2/P5 절과 같은 컨벤션(설계 근거, 코드 참조,
   실측치, 정직한 한계 고지)을 따랐다. Decoupled Tier 1 설계 근거, SE(2)
   변환 공식, `BaseFootprintChecker`/`drive_base_to_pose`의 검증 한계를
   명시.
2. 단순 설명 문서를 넘어, 실제로 `planning/reachability.py`·
   `planning/base_pose.py`를 그 자리에서 실행해 **진짜 데이터**를
   얻었다(가짜/예시 데이터를 그리지 않는다는 이 세션의 확립된 원칙) —
   수평 슬라이스(z=1.2m, 99점, 10.6초), 수직 슬라이스(y=-0.6m, 110점,
   9.9초), 그리고 `select_base_pose`가 실제로 먼 목표(1.6, -0.9, 1.2)를
   원점 베이스에서는 점수 0.124로 판정하고 재배치 후(1.25, -0.29, 60°)
   점수 1.0으로 바꾸는 재배치 시나리오.
3. 이 실측 데이터를 임베드한 단일 HTML 아티팩트(탭 4개: 개요/P7.0/P7.1/
   진행상태)를 `artifact-design`+`dataviz` 스킬 가이드에 따라 제작·발행.
   히트맵과 재배치 다이어그램은 canvas로 직접 그린다(라이브러리 불필요).

## What worked / what failed

실측 스크립트가 처음 예상(81초/504점, 기존 P7.0 문서 수치)보다 훨씬
빠르게 끝났다 — 격자를 성기게(0.15m)+슬라이스로 나누고 `n_restarts`를
8로 낮춘 덕분에 두 슬라이스(99+110점)가 20초 안에 끝났다. `select_base_pose`
용 전체 격자(coarse, step=0.25, 252점)도 25초 만에 끝나 총 실측 소요는
약 46초 — 아티팩트 하나를 위해 매번 이 정도 실측을 도는 게 부담스럽지
않다는 걸 확인했다(다음에 비슷한 요청이 오면 반복 가능).

## North-star delta

P7.0/P7.1이 코드로만 존재하던 상태에서 문서화 + 시각적 검증까지 완료됐다
— "베이스 배치를 통한 모바일 매니퓰레이션"의 첫 두 벽돌이 이제 다른
사람(또는 다음 cycle의 나 자신)이 코드를 안 읽고도 이해·신뢰할 수 있는
상태가 됐다.

## Key learnings

- **문서용 시각화라도 실측을 도는 비용이 생각보다 낮다.** 슬라이스로
  쪼개고 restart 수를 줄이면 전체 격자 재현 없이도 "진짜 데이터" 기준을
  지킬 수 있다 — 합성/가짜 예시 데이터로 타협할 필요가 없었다.
- **`success_rate`가 이산값이라는 P7.0의 알려진 한계가 히트맵에 그대로
  드러난다** — 경계 근처의 얼룩덜룩한 패턴(고립된 성공/실패 칸)이
  노이즈처럼 보이지만 실제로는 "여러 시드를 이미 소진한 단일 실행
  결과"라는 설계상 특성이다. 문서에도 아티팩트에도 이 점을 명시했다.

## Recommended next 1–3 priorities

1. PR #12(이번 문서화) 사람 리뷰/병합.
2. PR #3/#4/#5/#7/#8 여전히 밀려 있음 — 리뷰 큐 정리.
3. `scripts/demo_plan_mobile_manipulator.py` 엔드투엔드 데모(현재 라이브러리
   레이어만 있고 이를 엮는 통합 스크립트는 없음) — P7 완결성을 위한
   다음 자연스러운 단계.

## Artifacts

- 브랜치: `docs/p7-mobile-manipulator-guide` (PR #12)
- HTML 아티팩트: https://claude.ai/code/artifact/883aeefa-230e-4889-be7e-264b22c32e9c
  (탭: 개요 / P7.0 도달가능성 지도 / P7.1 베이스 자세 선택 / 진행 상태)
