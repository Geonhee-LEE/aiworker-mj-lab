# Research State — auto-generated each cycle

_Last updated: 2026-09-05 · cycle p7-1-base-pose_

## North star distance

P0·P1이 완료·병합됐다. P2·P4·P5는 구현·테스트 완료 상태로 리뷰 대기
(PR #3/#4/#5/#7). P3(정식 실행 모듈)도 완료됐다(PR #8). PR #9(PRD)와
#10(P7.0 reachability map)도 완료·병합됐다.

**PRD 범위가 P7(모바일 매니퓰레이터 계획)까지 확장됐다** — 사용자가
오른팔 단독이 아니라 베이스까지 포함한 IK·모션 플래닝 설계를 명시적으로
요청(2026-09-04). 조사 결과 이 로봇은 실제로 모바일(수동 평면 가상 관절 +
스워브 드라이브)이고, 반응형 whole-body IK(`control.whole_body.WholeBodyIK`)와
베이스 실행 계층(`control.base`)은 이미 있다 — 빠진 건 전역 모션 플래닝뿐이다.
실전 시스템의 표준 "decoupled" 패턴(reachability map 기반 베이스 배치 +
기존 팔 계획기 재사용)을 Tier 1으로 설계했다.

**P7.0(reachability map, PR #10)이 병합됐고, 이번 cycle에 P7.1(베이스
자세 선택)이 착수·완료됐다** — `planning/base_pose.py`(PR #11,
MP-0027). `world_to_base_frame`(월드→베이스 SE(2) 변환), `select_base_pose`
(reachability 점수·발자국 충돌·현재 위치 근접도로 후보 순위), 그리고
`ArmCollisionChecker`와 같은 아키텍처를 그대로 재현한 `BaseFootprintChecker`.
베이스 주행은 `WholeBodyIK`(손 목표 반응형이라 지점-대-지점 주행에 안
맞음) 대신 기존 `SwerveDrive`를 얇게 감싼 `planning/mobile_execution.py`로
처리 — 새 저수준 제어 없음. 핵심 회귀 테스트는 `build_reachability_map`을
그대로 재사용해 먼 베이스 위치에서는 진짜 IK로 도달 불가능했던 타겟이
`select_base_pose`가 고른 위치에서는 도달 가능해짐을 실증한다.

**열한 개 PR(#1~#11) 중 다섯(#1·#2·#9·#10·#11)이 병합됐고, 나머지
(#3/#4/#5/#7/#8)가 리뷰 대기**다. P7.0·P7.1이 모두 `main`에 있어
"reachability 기반 베이스 배치 + 기존 팔 계획기 재사용" decoupled 설계가
실제로 완성됐다.

## Current bottleneck

**PR #3/#4/#5/#7/#8 사람 리뷰/병합 대기** — 다섯 개가 쌓였다.
**PR #4·#5는 PR #1/#2 병합 여파로 `__init__.py`에서 충돌 상태**(export
합집합으로 수동 해결 필요, 절차는 이미 확립돼 있다).

## Open experiments

| Branch | Last update | Last description | Days open |
|---|---|---|---|
| planning/p2-demo-natural-motion | 2026-09-03 | MP-0023 데모 실행 경로 연결, PR #3 리뷰 대기 | 1 |
| planning/p5-rrt-star-planner | 2026-09-03 | MP-0016 RRT* 대안 플래너, PR #4 리뷰 대기(`__init__.py` 충돌 발생) | 1 |
| planning/chomp-posture-smoothing | 2026-09-03 | MP-0024 CHOMP류 궤적 최적화, PR #5 리뷰 대기(`__init__.py` 충돌 발생) | 1 |
| planning/p4-benchmark-harness | 2026-09-03 | MP-0013+MP-0004 벤치마크 하네스+첫 측정, PR #7 리뷰 대기 | 1 |
| planning/p3-execution-module | 2026-09-04 | MP-0008+MP-0009 P3 실행 모듈, PR #8 리뷰 대기 | 0 |
| planning/p7-1-base-pose | 2026-09-05 | MP-0027 P7.1 base_pose.py + mobile_execution.py, PR #11 리뷰 대기 | 0 |

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
- **`build_reachability_map`은 실제로 "베이스 원점 전용"이 아니다** —
  grid point는 내부적으로 그냥 절대 world IK 타겟으로 쓰인다("베이스
  원점" 요구사항은 결과를 나중에 재사용 가능한 상대 좌표로 해석하기
  위한 호출자 쪽 약속일 뿐). 그래서 P7.1 테스트에서 이 함수를 그대로
  재사용해 "임의의 베이스 위치에서 실제 IK로 도달 가능한가"를 검증할 수
  있었다 — 새 IK 검증 코드를 안 만들고도 진짜 회귀 증명이 가능했던 이유.
- **베이스 yaw를 "로봇이 목표 쪽을 정면으로 바라본다"고 가정하면 틀릴 수
  있다** — reachability 격자 실측이 +x보다 -y로 더 넓게 퍼져 있어(P7.0
  기본 격자 y∈[-1.1, 0.2]), 어느 축이 "정면"인지 모델에 새기지 않고
  같은 후보 각도 집합을 위치·방향 양쪽에 재사용해 전체 조합을 탐색하는
  쪽이 더 안전했다.
- **실제 장면(full_scene.xml can-sort)엔 베이스 발자국 높이대([0.27,
  0.51]m)에 겹치는 정적 장애물이 없다** — table은 z∈[0.63, 0.73]로 그
  위에 있다. `BaseFootprintChecker`의 참-충돌 테스트는 합성 MJCF로
  대신 검증했다 — 실제 장면에 낮은 장애물이 생기기 전까진 이 한계가
  유효하다.

## Next claude-actionable

1. PR #3/#4/#5/#7/#8/#11이 병합되면 `benchmark_planning.py`에
   `--planner`/`--postprocess` 플래그를 추가해 MP-0007/MP-0017 비교
   벤치마크로 확장.
2. P7 Tier 2(결합형 `WholeBodySpace`, 후속) — `select_base_pose` +
   고정-베이스 파이프라인의 decoupled 방식이 자리잡았으니, 우선순위가
   바뀌지 않는 한 급하지 않음.
3. 사용자가 우선순위를 정하면 나머지 두 한계(IK 실패 개선 — `MP-0011`;
   연속 동작 부드러움 — `MP-0021`) 중 하나를 이어서 진행.

## Next user-blocked

1. **PR #3/#4/#5/#7/#8/#11 사람 리뷰/병합** — PR #4·#5는 `__init__.py`
   충돌 해결이 필요(export 합집합으로 수동 병합, 절차는 확립됨).
2. **MP-0020** Telegram 봇 생성 및 `telegram_setup.sh` 실행 (사람만 가능).

## Cycles to date

16 (2026-08-30~09-05 사람 주도: P0 부트스트랩, P1 RRT-Connect 구현, 데모
반복/트리 시각화, 장애물 재배치, Q-space 시각화+CVD 팔레트, 인터랙티브 마우스
목표+버그 수정 3건, nullspace 정칙화+hydrax 조사, 데모 실행 경로에 shortcut+
시간 파라미터화 연결, RRT* 대안 플래너, CHOMP류 궤적 최적화 후처리, 벤치마크
하네스+P1 성공률 첫 측정, PR #1/#2 병합+P3 실행 모듈, PRD를 P7(모바일
매니퓰레이터)까지 확장 + P7.0 reachability map, P7.1 베이스 자세 선택; 자율
루프: shortcut 평활화, 시간 파라미터화)
