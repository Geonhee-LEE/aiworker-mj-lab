# Research State — auto-generated each cycle

_Last updated: 2026-09-05 · cycle p7-tier2-feasibility-and-pr-queue-clear_

## North star distance

**PR 큐가 전부 비워졌다.** 열두 개 PR(#1~#12)이 전부 병합됐다 — 오늘
남아 있던 다섯 개(#3/#4/#5/#7/#8)와 이번 cycle에 새로 연 #12까지 전부
사람 확인(AskUserQuestion) 후 실제로 병합·검증·push 완료했다. 지금
`main`에는 P0/P1/P2(shortcut+시간 파라미터화, 데모 연결까지)/P3(정식
실행 모듈)/P5(RRT* 대안 플래너 + CHOMP류 자세 평활화, 데모 연결까지)/
P7.0(reachability map)/P7.1(베이스 자세 선택 + 발자국 충돌 검사 + 얇은
주행 실행)이 전부 구현·병합돼 있다. `docs/guide/motion-planning.md`도
이 모든 걸 반영해 최신화됐다.

**P7 Tier 2(결합형 whole-body 플래너) 타당성 평가도 이번 cycle에
완료됐다** — 사용자가 "whole body control motion planning은 가능할지
계획 수립해주세요"라고 요청, plan mode + Explore 서브에이전트 3개(병렬)로
근거를 모아 **"가능하다"고 결론**냈다: 가장 어려운 부분(base+lift+arm
결합 11-DOF의 FK/Jacobian)이 이미 `control.whole_body.WholeBodyIK`+
`KinematicTree`로 프로덕션에서 검증돼 있고, `planning/`의 알고리즘 4개
(RRT-Connect/shortcut/trajectory/EdgeChecker)는 정말로 인터페이스 전용
이라 무수정 재사용 가능하다. 다만 세 가지 실제 수정이 필요하다:
① `ArmCollisionChecker`의 자기충돌 판정이 팔 전용 body 접두어로
하드코딩돼 있어 베이스/리프트 충돌을 조용히 놓칠 수 있음(생성자에
`planned_body_prefixes` 파라미터 추가 필요), ② `trajectory.py`의 시간
파라미터화가 모든 차원에 같은 스칼라 속도 상한을 써서 m/rad 혼합에
안 맞음, ③ 베이스 관절이 무제한이라 독자적인 샘플링 경계가 필요함
(`RightArmSpace.sample()`의 "unlimited면 0" 관례를 그대로 쓰면 베이스가
원점에 고정되는 조용한 버그). **결정적으로 지금 실제 씬엔 결합 계획이
필요한 시나리오가 없다**(정적 장애물이 전부 베이스 충돌 높이 위에 떠
있음) — 그래서 착수는 하지 않고 설계 문서만 `docs/guide/motion-planning.md`
"P7 Tier 2 타당성 평가" 절(PR #12)로 남겼다. `MP-0030`(owner=user)이
착수 여부 결정을 기다린다.

## Current bottleneck

**없음 — PR 큐가 처음으로 완전히 비었다.** 남은 열려 있는 항목은 전부
Backlog(13개)뿐이고 owner=user인 우선순위 결정 대기 항목(`MP-0020`
Telegram, `MP-0021` hydrax, `MP-0025` VAMP-MR, `MP-0030` P7 Tier 2)과
claude가 다음에 자연스럽게 집을 수 있는 항목(`MP-0007`/`MP-0011`/
`MP-0012`/`MP-0017`/`MP-0018`/`MP-0019`/`MP-0022`/`MP-0028`)이 섞여
있다.

## Open experiments

| Branch | Last update | Last description | Days open |
|---|---|---|---|

(없음 — 모든 작업 브랜치가 병합되고 로컬에서 삭제됐다.)

## Recent learnings (last 3 cycles)

- **여러 PR이 같은 문서 섹션을 독립적으로 추가하면 `git`이 감지 못 하는
  "의미론적 중복" 충돌이 생긴다.** PR #3과 PR #4가 각자
  `docs/guide/motion-planning.md`에 거의 동일한 "RRT-Connect 알고리즘
  요약" 절을 추가했는데, 두 절 사이에 다른 내용(PR #3의 "경로 후처리"
  절)이 끼어 있어서 git의 줄 단위 3-way merge가 이걸 충돌로 인식하지
  못하고 그냥 둘 다 살렸다 — `__init__.py`처럼 항상 뜨는 기계적 충돌과
  달리, 이런 건 병합 후 파일을 실제로 읽어야만 발견된다. 수동으로 중복
  절을 하나로 합치고 상호 참조를 갱신했다.
- **PR 순서를 신중하게 고르면 나중 충돌의 크기를 줄일 수 있다.** #7→
  #8→#3→#4→#5→#12 순서로 병합했다 — 문서를 안 건드리는 PR(#7)을 먼저,
  같은 파일을 건드리는 PR들(#3/#4)을 순서대로, 마지막에 내 자신의 문서
  PR(#12)을 배치해 "다음 단계" 문단 충돌이 최소 횟수로만 나게 했다.
- **`_run_interactive`처럼 시그니처가 여러 PR에서 각자 다르게 확장된
  함수는, 두 시그니처를 단순 union하는 게 아니라 실제로 어떤 하위
  호출(body)이 뭘 쓰는지 확인해야 한다.** git이 함수 본문은 이미
  올바르게 병합해 뒀는데(예: `_plan_path`와 `trajectory_settings`를
  둘 다 참조) 정의부만 두 개로 쪼개져 있었다 — 본문 기준으로 "어느
  시그니처가 실제로 맞는지" 역산해서 정의를 하나로 합쳤다.
- **CHOMP(PR #5)의 `_maybe_smooth_posture`와 P2(PR #3)의
  `_postprocess_path`는 독립적으로 만들어진 두 개의 "경로 후처리
  파이프라인"이었다** — 하나로 합치지 않고 그냥 나란히 두면 순서가
  꼬이거나(shortcut 전에 CHOMP를 적용하는 등) 한쪽이 무시된다. CHOMP
  단계를 `_postprocess_path` 내부(shortcut 다음, time_parameterize
  이전)에 끼워 넣어 한 파이프라인으로 통합했다 — 병합 후
  `--posture-smooth`/`--planner rrt_star`를 실제로 함께 실행해 자세
  매끄러움 비용이 실제로 줄어드는지(1.4254→0.0458) 확인했다.
- **Tier 2 타당성 평가에서: "인터페이스만 맞으면 무수정 재사용
  가능하다"는 결론은 공개 인터페이스의 절반만 본 것이었다.**
  `ArmCollisionChecker`가 내부적으로 "어떤 body가 계획 대상인가"를
  별도 하드코딩 목록(`_collect_planned_geoms`)으로 판정한다는 건
  `space`/`checker`의 공개 메서드 시그니처만 봐서는 안 보이는 부분 —
  구현 세부사항까지 읽어야 놓치는 버그(베이스 충돌이 조용히 무시되는
  경우)를 미리 잡을 수 있다.
- **문서용 시각화라도 실측을 도는 비용이 생각보다 낮다.** P7.0/P7.1
  HTML 요약 아티팩트를 위해 실제로 `reachability.py`/`base_pose.py`를
  실행해 진짜 데이터(히트맵, 재배치 시나리오)를 얻었다 — 슬라이스로
  쪼개고 restart 수를 줄이면 전체 격자 재현 없이도 "진짜 데이터" 기준을
  지킬 수 있었다(총 46초).

## Next claude-actionable

1. `benchmark_planning.py`에 `--planner`/`--postprocess` 플래그를 추가해
   `MP-0007`(shortcut 전/후 경로 길이 비교)/`MP-0017`(RRT-Connect vs
   RRT* 50-seed 비교표, `RESULTS.md`)을 실제로 측정 — PR 큐가 비었으니
   막힘 없이 바로 착수 가능.
2. `MP-0011`/`MP-0012`(IK 시드 다중 재시도 `planning/goals.py` + 중복
   제거) — 다음으로 자연스러운 claude-owner 백로그.
3. `MP-0018`/`MP-0019`(집계기 + todo_tool 단위 테스트) — 인프라 정리.

## Next user-blocked

1. **`MP-0030`** P7 Tier 2(결합형 whole-body 플래너) 착수 여부 결정 —
   타당성 평가 완료, 검증 시나리오(합성 MJCF)가 먼저 필요하다는 점까지
   문서화됨.
2. **`MP-0020`** Telegram 봇 생성 및 `telegram_setup.sh` 실행 (사람만
   가능).
3. **`MP-0021`**(hydrax)/`MP-0025`(VAMP-MR) 우선순위 결정 — 둘 다 조사는
   끝났고 구현 여부만 사용자 판단 대기.

## Cycles to date

17 (2026-08-30~09-05 사람 주도: P0 부트스트랩, P1 RRT-Connect 구현, 데모
반복/트리 시각화, 장애물 재배치, Q-space 시각화+CVD 팔레트, 인터랙티브 마우스
목표+버그 수정 3건, nullspace 정칙화+hydrax 조사, 데모 실행 경로에 shortcut+
시간 파라미터화 연결, RRT* 대안 플래너, CHOMP류 궤적 최적화 후처리, 벤치마크
하네스+P1 성공률 첫 측정, PR #1/#2 병합+P3 실행 모듈, PRD를 P7(모바일
매니퓰레이터)까지 확장 + P7.0 reachability map, P7.1 베이스 자세 선택,
P7 Tier 2 타당성 평가 + PR #3/#4/#5/#7/#8/#12 전체 병합(문서 중복 수동
정리 포함)로 PR 큐 완전 소진; 자율 루프: shortcut 평활화, 시간
파라미터화)
