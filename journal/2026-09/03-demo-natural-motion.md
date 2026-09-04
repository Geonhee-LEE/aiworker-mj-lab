# 데모 실행 경로에 shortcut+시간 파라미터화 연결 — 재생 컨트롤러 대역폭 문제 발견

- **Cycle**: 2026-09-03
- **Branch**: `planning/p2-demo-natural-motion`
- **TODO**: `MP-0022` shortcut+시간 파라미터화를 데모 실행 경로에 연결
- **Phase**: P2
- **Status**: keep

## What I tried

사용자가 `--interactive` 데모로 직접 목표를 옮겨 가며 확인한 결과, 팔이
목적지에는 도달하지만 경로 중간 자세가 부자연스럽다고 지적했다. 원인은
`_run_cycle`/`_run_interactive`가 `plan_rrt_connect`의 raw 경로(트리 확장의
지그재그 waypoint)를 그대로 재생하고 있었기 때문이다. 정확히 이 문제를 푸는
코드가 자동 연구 루프가 이미 구현·테스트까지 끝냈지만 main에 병합되지 않은
채 PR #1(MP-0005, shortcut)/PR #2(MP-0006, time_parameterize)로 대기 중이었다.

사용자가 두 PR을 지금 병합하지 않고 필요한 파일만 새 브랜치로 옮기는 쪽을
선택해, `git show <branch>:<path>`로 두 PR 브랜치의 최종 파일 내용을 그대로
복사했다(`shortcut.py`, `trajectory.py`, 각 테스트 파일, `settings.py`의
`TrajectorySettings`, `config/default.yaml`의 `planning.trajectory.*` 블록).
`__init__.py`는 두 PR의 export를 합집합으로 병합 — diff가 서로 다른 줄을
건드려 충돌 없음.

`_execute`를 waypoint 수렴-게이팅 방식에서 `Trajectory` 표본(물리 timestep
간격) 재생 방식으로 바꿨다. `--no-shortcut`/`--no-time-parameterize`
비교 플래그는 예전 방식(`_execute_waypoints`)을 그대로 보존해 raw 경로와
비교할 수 있게 했다.

## What worked / what failed

`--no-time-parameterize`(비교 모드)를 처음 구현했을 때, 시간 정보 없는
"가짜 Trajectory"를 만들어 새 `_execute`에 그대로 넘기는 실수를 했다 — waypoint당
물리 스텝 1개만 진행해 버려서 최종 오차가 0.83 rad까지 벌어졌다. 별도
`_execute_waypoints`(예전 수렴-게이팅 로직 보존)로 분리해 고쳤다.

더 중요한 발견: 기본(`--no-time-parameterize` 없이) 경로에서도 config의
`planning.trajectory.max_joint_speed_rad_s`(4.8 rad/s, FFW-SH5 실제 하드웨어
관절 한계)를 그대로 재생에 쓰면 문제가 생겼다. 진단 스크립트로 실측: 이
데모가 재생에 쓰는 `ArmTorqueController`(오픈루프 PD + 중력보상 토크
제어기)는 4.8 rad/s 기준 궤적을 전혀 따라가지 못하고, 중간 추종 오차가
최대 1.5 rad까지 계속 벌어졌다(궤적이 멈춘 뒤에야 서서히 수렴). 속도를
1.0 rad/s로 낮추자 중간 오차가 0.12 rad, 최종 오차가 기존과 같은 0.02 rad
수준으로 돌아왔다. `--exec-max-speed-rad-s`(기본 1.0)/`--exec-max-accel-rad-s2`
(기본 2.0) CLI 플래그를 새로 추가해 재생 전용 속도를 config의 하드웨어
한계와 분리했다 — config 값 자체는 바꾸지 않았다(실제 하드웨어 스펙을
잘못 기록하게 되는 셈이라). `MP-0008`(P3 실행 모듈, `ArmTorqueController`
연결)이 아직 없어서 `Trajectory`를 실제로 소비하는 첫 지점이 이번이었고,
그래서 이 대역폭 불일치가 지금까지 발견되지 않았던 것으로 보인다.

## North-star delta

P2(경로 후처리)가 데모 실행 경로에 실제로 연결됐다 — shortcut 평활화로 경로
길이가 seed별로 5~40% 줄고(실측: 4.640→4.256, 13.616→8.217 rad 등), 시간
파라미터화로 매끄러운 궤적 재생이 가능해졌다. 사용자가 지적한 "중간 경로가
부자연스럽다"는 문제의 직접적인 수정이다. 43개 planning 테스트(신규 shortcut/
trajectory 테스트 포함) 통과.

## Key learnings

- **하드웨어 관절 한계 ≠ 시뮬레이션 재생 컨트롤러가 실제로 추종 가능한
  속도.** `planning.trajectory.max_joint_speed_rad_s`는 실제 로봇(자체
  서보/저수준 제어기가 있는)이 궤적을 재생할 때는 맞는 값이지만, 이 저장소의
  `ArmTorqueController`처럼 오픈루프 PD+중력보상만으로 목표 위치를 쫓는
  컨트롤러에는 너무 빠르다. **하드웨어 스펙과 "이 컨트롤러가 실제로
  추종 가능한 속도"는 서로 다른 값으로 분리해서 관리해야 한다** — 하나를
  다른 하나에 맞춰 낮추면 스펙 자체가 왜곡된다.
  `test_trajectory_speed_matches_hardware_joint_limit`가 검증하는 건
  "config가 문서화한 하드웨어 스펙 일관성"이지 "이 컨트롤러가 그 속도로
  추종 가능한지"가 아니다 — 이 둘을 혼동하면 안 된다.
- **`Trajectory`를 처음 실제로 소비하는 코드가 이 실측을 처음 드러냈다** —
  PR #2의 자체 테스트는 순수 kinematic 성질(속도·가속도 상한 자체를 지키는
  파라미터화인지)만 검증했고, 실제 토크 제어기가 그 파라미터화를 추종할 수
  있는지는 검증 범위 밖이었다. 정식 P3(`planning.execution`, MP-0008)를
  만들 때 이 발견을 반영해야 한다 — 예: 재생 컨트롤러의 대역폭을 실측해
  기본 속도 상한으로 문서화하거나, 추종 오차 기반 적응형 속도 조절을 고려.

## Recommended next 1–3 priorities

1. PR #1(MP-0005)·PR #2(MP-0006) 사람 리뷰/병합 — 이 브랜치는 그 내용을
   앞당겨 썼을 뿐 대체하지 않는다.
2. `MP-0008`(P3 실행 모듈) 설계 시 이번에 발견한 "재생 컨트롤러 대역폭 ≠
   하드웨어 스펙" 구분을 반영.
3. `planning/p5-rrt-star-planner`(RRT* 대안 플래너, MP-0016) — 같은 세션에서
   이어서 진행.

## Artifacts

- 브랜치: `planning/p2-demo-natural-motion`
- 진단 스크립트(비체크인, 세션 스크래치): 재생 속도 스윕으로 4.8/2.0/1.0/0.5
  rad/s 각각의 중간·최종 추종 오차 실측
