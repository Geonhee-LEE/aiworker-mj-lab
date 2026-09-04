# Research State — auto-generated each cycle

_Last updated: 2026-09-04 · cycle p7-reachability-map_

## North star distance

P0·P1이 완료·병합됐다. P2·P4·P5는 구현·테스트 완료 상태로 리뷰 대기
(PR #3/#4/#5/#7). P3(정식 실행 모듈)도 완료됐다(PR #8).

**PRD 범위가 P7(모바일 매니퓰레이터 계획)까지 확장됐다** — 사용자가
오른팔 단독이 아니라 베이스까지 포함한 IK·모션 플래닝 설계를 명시적으로
요청(2026-09-04). PR #9(docs)가 Non-Goals에서 베이스·리프트를 제외하고
로드맵에 P7을 추가했다. 조사 결과 이 로봇은 실제로 모바일(수동 평면
가상 관절 + 스워브 드라이브)이고, 반응형 whole-body IK
(`control.whole_body.WholeBodyIK`)와 베이스 실행 계층(`control.base`)은
이미 있다 — 빠진 건 전역 모션 플래닝뿐이다. 실전 시스템의 표준
"decoupled" 패턴(reachability map 기반 베이스 배치 + 기존 팔 계획기
재사용)을 Tier 1으로 설계했다.

**P7.0(reachability map)이 이번 cycle에 착수·완료됐다** —
`planning/reachability.py`(PR #10, MP-0026). 새 IK를 만들지 않고 기존
`_ik_attempt`/`_solve_valid_ik` 패턴을 재사용. 실측: 기본 격자(504점)
전체 빌드 81초(로봇 1대당 1회 캐시하는 오프라인 아티팩트로는 허용
범위), 도달 가능/불가능 지점이 실제로 갈리는 것 확인.

**열 개 PR(#1~#10) 중 아홉(#1·#2는 병합됨, 나머지 #3/#4/#5/#7/#8/#9/#10
리뷰 대기)**이다.

## Current bottleneck

**PR #3/#4/#5/#7/#8/#9/#10 사람 리뷰/병합 대기** — 일곱 개가 쌓였다.
**PR #4·#5는 PR #1/#2 병합 여파로 `__init__.py`에서 충돌 상태**(export
합집합으로 수동 해결 필요, 절차는 이미 확립돼 있다). PR #9(PRD)와
#10(reachability)은 서로 독립이지만, PR #9가 먼저 병합돼야 PRD 문서가
P7을 공식적으로 인정한 상태가 된다.

## Open experiments

| Branch | Last update | Last description | Days open |
|---|---|---|---|
| planning/p2-demo-natural-motion | 2026-09-03 | MP-0023 데모 실행 경로 연결, PR #3 리뷰 대기 | 1 |
| planning/p5-rrt-star-planner | 2026-09-03 | MP-0016 RRT* 대안 플래너, PR #4 리뷰 대기(`__init__.py` 충돌 발생) | 1 |
| planning/chomp-posture-smoothing | 2026-09-03 | MP-0024 CHOMP류 궤적 최적화, PR #5 리뷰 대기(`__init__.py` 충돌 발생) | 1 |
| planning/p4-benchmark-harness | 2026-09-03 | MP-0013+MP-0004 벤치마크 하네스+첫 측정, PR #7 리뷰 대기 | 1 |
| planning/p3-execution-module | 2026-09-04 | MP-0008+MP-0009 P3 실행 모듈, PR #8 리뷰 대기 | 0 |
| docs/prd-mobile-manipulator-scope | 2026-09-04 | PRD를 P7까지 확장, PR #9 리뷰 대기 | 0 |
| planning/p7-reachability-map | 2026-09-04 | MP-0026 P7.0 reachability map + todo_tool.py P7 지원, PR #10 리뷰 대기 | 0 |

## Recent learnings (last 3 cycles)

- **기존 플래너가 "팔 전용"이 아니라 "공간 인터페이스 무관"이라는 걸
  확인했다.** `plan_rrt_connect`/`plan_rrt_star`/`shortcut_path`/
  `smooth_posture`는 전부 `space`(`.sample`/`.distance`/`.steer`/
  `.interpolate`/`.contains`/`.write`)와 `checker.is_valid`라는 추상
  인터페이스에만 의존한다 — 이 인터페이스를 만족하는 베이스+팔 공간을
  하나 더 만들면(P7 Tier 2, 후속) 알고리즘 코드를 한 줄도 안 고쳐도
  된다. 좋은 추상화가 미래 확장을 얼마나 값싸게 만드는지 보여주는 사례.
- **모바일 매니퓰레이터 계획의 표준 실전 패턴은 완전 결합이 아니라
  decoupled(베이스 배치 후 팔만 계획)다** — 외부 조사(Fetch/TIAGo/HSR,
  PickNik 등)로 확인. 이미 존재하는 반응형 whole-body IK/베이스 실행
  계층을 재사용하면서 전역 계획만 새로 추가하는 게 이 저장소의 "기존
  자산 재사용 우선" 원칙과도 정확히 맞아떨어졌다.
- **공유 워킹 디렉토리에서 `git checkout -b`는 실패해도 조용하다 —
  반드시 직후에 `git branch --show-current`로 확인해야 한다.** 이
  세션에 반복된 실수라 이번 cycle엔 매 브랜치 전환마다 확인해 사고
  없이 진행했다 — 습관화가 실제로 통했다.
- **PRD가 요구하는 값(예: TODO Phase 열)이 하드코딩된 enum(예:
  `todo_tool.py`의 `PHASES` 튜플)과 어긋나면 도구가 조용히 새 값을
  거부한다** — 새 Phase를 추가할 땐 PRD뿐 아니라 그걸 검증하는 도구도
  같이 고쳐야 한다.

## Next claude-actionable

1. PR #3/#4/#5/#7/#8이 병합되면 `benchmark_planning.py`에
   `--planner`/`--postprocess` 플래그를 추가해 MP-0007/MP-0017 비교
   벤치마크로 확장.
2. **MP-0027** P7.1 `planning/base_pose.py` — PR #9·#10 병합 후 착수.
   reachability map으로 베이스 자세 선택 + 베이스 발자국 충돌 검사 +
   기존 팔 계획기와 end-to-end 데모.
3. 사용자가 우선순위를 정하면 나머지 두 한계(IK 실패 개선 — `MP-0011`;
   연속 동작 부드러움 — `MP-0021`) 중 하나를 이어서 진행.

## Next user-blocked

1. **PR #3/#4/#5/#7/#8/#9/#10 사람 리뷰/병합** — PR #4·#5는 `__init__.py`
   충돌 해결이 필요(export 합집합으로 수동 병합, 절차는 확립됨).
2. **MP-0020** Telegram 봇 생성 및 `telegram_setup.sh` 실행 (사람만 가능).

## Cycles to date

15 (2026-08-30~09-04 사람 주도: P0 부트스트랩, P1 RRT-Connect 구현, 데모
반복/트리 시각화, 장애물 재배치, Q-space 시각화+CVD 팔레트, 인터랙티브 마우스
목표+버그 수정 3건, nullspace 정칙화+hydrax 조사, 데모 실행 경로에 shortcut+
시간 파라미터화 연결, RRT* 대안 플래너, CHOMP류 궤적 최적화 후처리, 벤치마크
하네스+P1 성공률 첫 측정, PR #1/#2 병합+P3 실행 모듈, PRD를 P7(모바일
매니퓰레이터)까지 확장 + P7.0 reachability map; 자율 루프: shortcut 평활화,
시간 파라미터화)
