# PRD를 모바일 매니퓰레이터(P7)까지 확장 + P7.0 reachability map

- **Cycle**: 2026-09-04
- **Branch**: `docs/prd-mobile-manipulator-scope`, `planning/p7-reachability-map`
- **TODO**: (PRD는 docs-only), `MP-0026`
- **Phase**: P7
- **Status**: keep

## What I tried

사용자가 오른팔 단독이 아니라 모바일 매니퓰레이터(베이스+팔) 전체에 대한
IK·모션 플래닝 설계를 요청 — 이는 PRD Non-Goals("왼팔·베이스·리프트 계획
...이 PRD 범위 밖")를 명시적으로 넘어서는 요청이라, 먼저 두 개의 Explore
서브에이전트를 병렬로 돌려 (1) 이 로봇이 실제로 모바일인지·기존
베이스/whole-body 코드가 있는지 (2) 실전 모바일 매니퓰레이터 IK·플래닝
접근법을 조사했다.

핵심 발견: 로봇은 실제로 모바일이다 — `base_x`/`base_y`(slide)/`base_yaw`
(hinge)로 이루어진 수동(액추에이터 없는) 평면 가상 관절이 3-모듈 스워브
드라이브의 바퀴-지면 마찰로 구동된다(사실상 홀로노믹). 그리고 이 저장소는
이미 반응형 whole-body IK(`control.whole_body.WholeBodyIK`, base+lift+
양팔을 하나의 weighted bounded differential IK로 묶음)와 베이스 실행
계층(`control.base`, 스워브 기구학+조향 상태기계)을 갖고 있었다 — 재구현할
필요가 전혀 없었다. 외부 조사 결론: 실전 시스템(Fetch/TIAGo/HSR,
PickNik/MoveIt Pro)은 완전 결합(coupled) 샘플링 계획을 기본으로 안 쓰고
"reachability map으로 베이스 배치 → 기존 고정-베이스 팔 계획기 재사용"
(decoupled)을 기본으로 쓴다.

이 통찰로 2-tier 설계를 했다: Tier 1(decoupled, 우선)은 기존 자산을
최대한 재사용, Tier 2(coupled `WholeBodySpace`)는 기존 RRT-Connect/
RRT*/shortcut/CHOMP가 전부 "팔 전용"이 아니라 추상 `space`/`checker`
인터페이스에만 의존한다는 걸 재확인해 후속 과제로 설계만 해 뒀다(지금은
실측 없이 만들지 않음 — "+50 LOC 정당화" 원칙).

PRD를 먼저 갱신(PR #9): Non-Goals에서 베이스·리프트 제외(왼팔은 유지),
로드맵에 P7 추가, R-F-011(reachability map)·R-F-012(베이스 자세 선택)
신규. 그다음 P7.0 `planning/reachability.py`(PR #10) 구현 — 새 IK를
만들지 않고 `_ik_attempt`/`_solve_valid_ik` 패턴 재사용, `home` 키프레임
에서 `base_x=base_y=base_yaw=0`임을 실측 확인해 이 장면의 월드 좌표가 곧
베이스 프레임 좌표임을 검증(변환 없이 그대로 사용 가능), 격자 경계는
3000개 무작위 유효 표본 FK 위치 1~99 백분위 실측으로 근거를 뒀다.

## What worked / what failed

`todo_tool.py check`가 TODO.md에 `Phase=P7` 행을 넣자 "잘못된 Phase"로
거부했다 — `PHASES` 튜플이 P6까지 하드코딩돼 있었다. PR #10 브랜치에
후속 커밋으로 고쳤다(별도 PR을 안 만들고 관련 작업에 자연스럽게 묶음).
이 과정에서 main의 우선 TODO.md 편집분을 코드 브랜치로 그대로 가져가지
않으려고 `git stash push -- TODO.md`로 딱 그 파일만 분리했다가 나중에
main으로 돌아와 `stash pop`으로 복원 — 지난 cycle들에서 반복됐던 "브랜치
전환 중 다른 파일이 딸려가는" 사고를 이번엔 목적을 갖고 stash로 예방했다.

세션에서 반복됐던 `git checkout -b` 조용한 실패 문제는 이번 cycle엔
매번 직후 `git branch --show-current`로 확인하는 습관을 지켜 한 번도
안 겪었다 — 3번 연속 사고 뒤 실제로 습관이 붙었다.

## North-star delta

PRD 범위가 P7까지 공식적으로 넓어졌다(병합 대기). P7.0(reachability map)
구현·테스트 완료로 "베이스 배치를 통한 모바일 매니퓰레이션"의 첫 벽돌이
놓였다 — 다음(P7.1, `base_pose.py`)이 이 지도를 실제로 써서 베이스를
옮기고 기존 팔 계획기와 이어붙이면 end-to-end 모바일 매니퓰레이션 데모가
완성된다.

## Key learnings

- **좋은 추상화는 미래 확장을 값싸게 만든다.** RRT-Connect/RRT*/
  shortcut/CHOMP를 짤 때 "팔 전용"이 아니라 "space/checker 인터페이스에만
  의존"하도록 설계했던 덕분에, 이번에 베이스+팔로 확장하는 설계(Tier 2)가
  "새 space 클래스 하나만 더 만들면 됨"으로 정리됐다 — 처음부터 의도한 건
  아니었지만 좋은 설계 습관이 우연히 보상받은 사례.
- **PRD Non-Goal을 넘어서는 사용자 요청은 PRD를 먼저 고치고 시작해야
  다음 cycle(자동 루프 포함)이 헷갈리지 않는다.**
- **새 enum 값(Phase 등)을 문서에 추가할 때 그걸 검증하는 도구
  (`todo_tool.py`)도 같이 고쳐야 한다** — 문서와 코드가 각자 다른 곳에서
  "유효한 값 목록"을 정의하고 있으면 어느 한쪽만 고치기 쉽다.

## Recommended next 1–3 priorities

1. PR #9(PRD)·#10(reachability map) 사람 리뷰/병합.
2. **MP-0027** P7.1 `planning/base_pose.py` — reachability map 소비,
   베이스 발자국 충돌 검사, end-to-end 데모.
3. PR #3/#4/#5/#7/#8(이전 cycle들의 밀린 PR) 리뷰/병합 — 큐가 7개로
   늘었다.

## Artifacts

- 브랜치: `docs/prd-mobile-manipulator-scope`(PR #9),
  `planning/p7-reachability-map`(PR #10)
- 실측: reachability map 기본 격자(504점, n_restarts=10) 빌드 81초,
  도달 가능/불가능 지점 분리 확인
