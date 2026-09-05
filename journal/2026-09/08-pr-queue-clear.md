# PR 큐 완전 소진 (#3/#4/#5/#7/#8/#12) + STATE/TODO 정리

- **Cycle**: 2026-09-05
- **Branch**: (main에 직접, 각 PR 병합 커밋)
- **TODO**: MP-0023, MP-0016, MP-0024, MP-0013, MP-0004, MP-0008, MP-0009, MP-0010, MP-0029
- **Phase**: P2/P3/P5/P7
- **Status**: keep

## What I tried

사용자가 "마저 진행해주세요"라고 요청 — P7 Tier 2 타당성 평가를 끝낸
직후라, 밀려 있던 PR 큐(#3/#4/#5/#7/#8 + 이번에 새로 연 #12) 정리를
다음 자연스러운 작업으로 판단했다.

각 PR을 `git merge --no-commit --no-ff`로 먼저 dry-run해 실제 충돌
범위를 확인한 뒤, `AskUserQuestion`으로 사용자에게 "전부 병합, 문서
중복은 내가 정리"를 승인받고 순서대로(#7→#8→#3→#4→#5→#12) 실제 병합을
진행했다. 매 병합마다: `__init__.py` 충돌은 export 합집합으로 해결,
테스트(`tests/test_planning*.py` + `test_config.py`) 재검증, ruff 린트,
필요하면 데모 스크립트 실행으로 기능 확인 후 push.

## What worked / what failed

**PR #3과 #4가 `docs/guide/motion-planning.md`에 거의 동일한
"RRT-Connect 알고리즘 요약" 절을 독립적으로 추가**해 뒀다는 걸 dry-run
단계에서 미리 발견 — git의 줄 단위 merge는 이 의미론적 중복을 감지하지
못하고(두 절 사이에 PR #3의 다른 내용이 끼어 있어 서로 다른 hunk로
인식) 그냥 둘 다 남겨서 문서에 같은 섹션이 두 번 나오게 됐을 거다.
사용자에게 미리 알리고 병합 순서를 정한 뒤, 실제 병합 시점에 중복
절을 수동으로 하나로 합치고 "RRT*는 TODO의 MP-0015/16/17로 진행 중"
같은 이제는 틀린 참조도 실제 상태(구현 완료)로 갱신했다.

**`scripts/demo_plan_right_arm.py`의 `_run_interactive` 함수가 PR #4/#5
양쪽에서 시그니처가 확장돼 있었다** — PR #3/#4가 합쳐지며 이미
`trajectory_settings` 파라미터가 붙어 있었는데, PR #5(CHOMP)는 그
확장 이전 시그니처를 기준으로 만들어져 있어 두 정의가 충돌했다.
`_plan_path`(PR#4가 새로 만든 dispatch 함수)와 `_maybe_smooth_posture`
(PR#5가 새로 만든 CHOMP 후처리 헬퍼)를 둘 다 살리고, `_run_interactive`
정의는 이미 병합된 함수 본문이 실제로 요구하는 시그니처(둘 다 필요)로
통일했다.

**CHOMP의 `_maybe_smooth_posture`와 기존 `_postprocess_path`가 서로
다른 두 개의 독립적인 "경로 후처리 파이프라인"이었다** — 나란히 두면
순서가 안 맞거나 한쪽이 무시될 위험이 있어, CHOMP 단계를
`_postprocess_path` 내부(shortcut 다음, time_parameterize 이전)에
끼워 넣어 하나의 파이프라인으로 통합했다. 병합 후
`--posture-smooth`/`--planner rrt_star`를 실제로 같이 실행해 조합이
멀쩡히 동작하고 자세 매끄러움 비용이 실제로 줄어드는지(1.4254→0.0458)
확인했다.

## North-star delta

PR 큐가 이 세션 시작 이래 처음으로 완전히 비었다 — P0~P3, P5, P7.0/
P7.1이 전부 `main`에 있고 서로 잘 맞물려 동작한다(RRT-Connect/RRT*
+ shortcut/CHOMP 후처리 + 시간 파라미터화 + 실행 모듈까지 엔드투엔드로
검증됨).

## Key learnings

- **여러 PR이 같은 문서 섹션을 독립적으로 추가하면 git이 감지 못 하는
  "의미론적 중복" 충돌이 생긴다.** 다른 내용이 두 절 사이에 끼어 있으면
  git의 줄 단위 3-way merge가 별개 hunk로 인식해 충돌 표시를 안 한다 —
  병합 dry-run 때 실제로 파일을 읽어야만 발견된다.
- **함수 시그니처가 여러 PR에서 각자 확장되면, 두 시그니처를 단순
  union하지 말고 이미 병합된 본문이 실제로 뭘 요구하는지 역산해야
  한다.** git이 본문(호출부)은 올바르게 합쳐 놨는데 정의부만 두 개로
  남는 경우가 있다.
- **독립적으로 개발된 두 "후처리 파이프라인"은 나란히 두지 말고 하나로
  통합해야 순서 문제를 피할 수 있다** — 병합 후 조합을 실제로 실행해
  검증하는 게 중요하다(테스트 통과만으로는 두 opt-in 플래그의 상호작용
  버그를 못 잡을 수 있다).

## Recommended next 1–3 priorities

1. `MP-0007`/`MP-0017` — 이제 벤치마크 하네스가 있으니
   `benchmark_planning.py`에 `--planner`/`--postprocess` 플래그를 추가해
   실제 비교 측정.
2. `MP-0011`/`MP-0012` — IK 시드 다중 재시도 정식화.
3. 사용자가 `MP-0030`(P7 Tier 2)/`MP-0021`(hydrax)/`MP-0025`(VAMP-MR)
   중 우선순위를 정하면 그에 따라 진행.

## Artifacts

- 병합 커밋: `82ac552`(#7), `d8437a3`(#8), `9878a9c`(#3), `80c4619`(#4),
  `ee4f2df`(#5), `23e8ad9`(#12)
- 정리 커밋: `52ae26b`(TODO.md)
