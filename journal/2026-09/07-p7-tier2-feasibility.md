# P7 Tier 2(결합형 whole-body 플래닝) 타당성 평가

- **Cycle**: 2026-09-05
- **Branch**: `docs/p7-mobile-manipulator-guide` (PR #12에 추가 커밋)
- **TODO**: `MP-0030`
- **Phase**: P7
- **Status**: keep

## What I tried

사용자가 "whole body control motion planning은 가능할지 계획
수립해주세요"라고 요청 — Tier 1(decoupled) 설계 때 후속으로 미뤄뒀던
완전 결합형(coupled) 11-DOF(base+lift+arm) 샘플링 플래닝의 실현
가능성을 판단해 달라는 것. `AskUserQuestion`으로 범위를 "타당성 평가 +
설계 문서만, 구현은 안 함"으로 확정한 뒤 plan mode로 진입해 Explore
서브에이전트 3개(병렬)로 근거를 모았다:

1. `KinematicTree`/`WholeBodyIK`(FK·Jacobian 인프라)
2. `planning/` 알고리즘 4개(rrt_connect/shortcut/trajectory/EdgeChecker)의
   인터페이스 순수성 + `ArmCollisionChecker`/`obstacles.py`의 정확한 계약
3. 실제 씬의 장애물 배치(결합 계획이 필요한 시나리오 존재 여부) +
   `control/base.py`/`mobile_execution.py`의 실행 계층 재사용 가능성

결과를 `docs/guide/motion-planning.md`에 새 절("P7 Tier 2 타당성 평가 —
설계 문서, 미착수")로 정리해 PR #12(이미 P7.0/P7.1을 문서화한 그 PR)에
추가 커밋으로 얹었다. 새 파일·코드 변경 없음 — 순수 문서.

## What worked / what failed

가장 큰 발견은 "예상보다 리스크가 낮다"였다 — Tier 1 설계 당시엔 Tier 2를
"완전 결합 SE(2)×Rⁿ 샘플링, 아직 미지수"로 남겨뒀는데, 실제로 코드를
읽어보니 `WholeBodyIK`가 지금 이 순간에도 `KinematicTree` 하나로 정확히
그 11+7(양팔) DOF Jacobian을 매 프레임 계산하고 있었다 — 가장 어려운
부분이 이미 프로덕션 검증을 거친 상태였다.

반대로 "완전 무수정 재사용" 가정은 세 군데에서 깨졌다(이전 Tier 1 초안이
낙관적으로 넘겼던 부분): `ArmCollisionChecker`의 자기충돌 판정이 팔 전용
body 접두어로 하드코딩돼 있어 베이스/리프트 충돌이 조용히 무시되는
문제, `trajectory.py`의 단일 스칼라 속도 상한이 m/rad 혼합 차원에 안
맞는 문제, `RightArmSpace.sample()`의 "unlimited면 0" 관례가 베이스를
원점에 고정시켜 버리는 문제. 셋 다 코드를 실제로 읽지 않았으면 놓쳤을
것들이다.

그리고 실제 씬(`full_scene.xml`)엔 결합 계획이 필요한 장애물이 아예
없다는 것도 확인했다 — 테이블/빈이 전부 베이스 충돌 높이대 위에 떠
있다. 이건 "지금 당장 만들 필요는 없다"는 근거를 더 단단하게 만들었다.

## North-star delta

Tier 2가 "언젠가 필요하면 그때 설계"에서 "이미 상세 청사진이 있고,
착수 조건(합성 검증 시나리오)만 갖추면 시작 가능"으로 바뀌었다 — 실제
착수는 사용자의 우선순위 판단(MP-0030)을 기다린다.

## Key learnings

- **"기존 자산 재사용"을 문서에만 쓰지 말고 실제로 그 자산을 쓰는 코드를
  한 줄씩 읽어야 진짜 타당성 평가가 된다.** `WholeBodyIK`가 이미
  `KinematicTree`로 베이스+팔 결합 FK를 하고 있다는 사실은 이전
  Tier 1 조사에서도 언급됐지만("반응형 whole-body IK는 이미 있다"),
  그게 **Tier 2가 필요로 하는 정확히 같은 FK 인프라**라는 연결은 이번에
  코드를 직접 대조하기 전까진 명시적으로 확인되지 않았었다.
- **"인터페이스만 맞으면 무수정 재사용 가능하다"는 결론은 인터페이스
  계약의 절반만 본 것이었다.** `space`/`checker`의 공개 메서드 시그니처는
  전부 인터페이스 그대로였지만, `ArmCollisionChecker`가 내부적으로
  "어떤 body가 계획 대상인가"를 별도 하드코딩 목록으로 판정한다는 건
  공개 인터페이스만 봐서는 안 보이는 부분이었다 — 구현 세부사항까지
  읽어야 놓치는 버그를 미리 잡을 수 있다.

## Recommended next 1–3 priorities

1. PR #12(P7.0/P7.1 문서 + 이번 Tier 2 타당성 평가) 사람 리뷰/병합.
2. PR #3/#4/#5/#7/#8 여전히 밀려 있음 — 리뷰 큐 정리.
3. 사용자가 Tier 2 착수를 결정하면(MP-0030), 첫 단계는 합성 MJCF 검증
   시나리오 작성 → `WholeBodySpace` 구현 순서로 진행.

## Artifacts

- 브랜치: `docs/p7-mobile-manipulator-guide` (PR #12, 이번 커밋
  `95a21e1`)
- 계획 파일: `/home/geonhee/.claude/plans/joyful-mixing-cascade.md`
  (타당성 평가 전문 + 설계 청사진)
