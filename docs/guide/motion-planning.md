# 오른팔 모션 플래닝

`src/ffw_sh5_grasp/planning/`은 오른팔 7-DOF(`arm_r_joint1..7`)를 위한
sampling-based 모션 플래너다. 베이스·리프트·헤드·손가락·왼팔은 계획 대상이
아니라 질의 시점의 상태로 고정한 장애물로 취급한다.

**P7(모바일 매니퓰레이터)부터는 베이스도 계획 대상에 들어온다** — 아래
[모바일 매니퓰레이터 P7 (베이스 + 오른팔)](#모바일-매니퓰레이터-p7-베이스--오른팔)
절 참고. 왼팔은 여전히 이 문서의 범위 밖이다.

## 설계 원칙

- **전역 플래너와 지역 반응층을 분리한다.** 이 모듈은 "충돌 없는 관절
  waypoint 열"을 만드는 데까지만 책임진다. 실행은 기존
  [`control.arm.ArmTorqueController`](../guide/arm_control.md)가 하고,
  실행 중 예상 못 한 접촉은 기존 whole-body IK의 collision CBF가 계속 담당한다.
  `WholeBodyIK`와 ACT 파이프라인은 건드리지 않는다.
- **기존 자산을 재사용한다.** FK는 `KinematicTree`, 관절 범위 클리핑 개념은
  `kinematics.constraints`와 같은 패턴을 따른다.

## 현재 구현 (P0 + P1)

| 모듈 | 책임 |
|---|---|
| `arm_state.RightArmSpace` | 오른팔 관절 이름·id·qpos 주소·범위. 샘플링·보간·클리핑 |
| `obstacles.right_arm_collision_pairs` | `clearance()`의 정확한 거리 보고에 쓰는 충돌 쌍 목록 |
| `collision_state.ArmCollisionChecker` | boolean `is_valid(q)` + exact `clearance(q)` |
| `local_path.EdgeChecker` | 두 configuration 사이 선분의 충돌 검사(이분 순서) |
| `settings.load_collision_settings` | `config/default.yaml`의 `planning.collision.*` 로더 |
| `rrt_connect.plan_rrt_connect` | 두 트리 EXTEND/CONNECT 표준 RRT-Connect. 결정론적 seed. 반환값에 `TreeSnapshot`(탐색한 전체 트리)도 포함 |

평활화·시간 파라미터화(P2)와 정식 실행 모듈(P3)은 아직 없다.

## 직접 실행해 보기

`scripts/demo_plan_right_arm.py`가 계획 → (선택) MuJoCo 물리 재생 → (선택)
실시간 뷰어까지 엔드투엔드로 보여준다.

```bash
# 계획만 (수 밀리초, 렌더링 없음)
PYTHONPATH=src MUJOCO_GL=osmesa python3 scripts/demo_plan_right_arm.py

# 계획 + 물리 재생까지 확인(관절 오차 출력)
PYTHONPATH=src MUJOCO_GL=osmesa python3 scripts/demo_plan_right_arm.py --execute

# 계획 + 재생을 실시간 뷰어 창으로 직접 확인 (디스플레이가 있는 환경에서).
# MUJOCO_GL은 설정하지 않는다 — osmesa/egl은 오프스크린 백엔드라 이 창형
# 뷰어(GLFW)와 충돌한다. 셸에 이미 export되어 있다면 -u로 지워야 한다.
PYTHONPATH=src env -u MUJOCO_GL python3 scripts/demo_plan_right_arm.py --execute --viewer

# 다른 목표로 반복(시드만 바꾸면 다른 무작위 유효 목표를 계획한다)
PYTHONPATH=src python3 scripts/demo_plan_right_arm.py --seed 3 --execute

# 목표에 도착할 때마다 새 무작위 목표로 계속 반복 (--loop 0 = 창을 닫을 때까지 무한 반복)
PYTHONPATH=src env -u MUJOCO_GL python3 scripts/demo_plan_right_arm.py --execute --viewer --loop 5

# RRT-Connect가 탐색한 두 트리(시작 쪽 초록, 목표 쪽 파랑)를 실시간 뷰어에 그린다
PYTHONPATH=src env -u MUJOCO_GL python3 scripts/demo_plan_right_arm.py --execute --show-tree --loop 3

# teleop_app.py처럼 목표를 마우스로 직접 옮긴다 — 노란 구슬을 더블클릭으로
# 선택하고 Ctrl+마우스 오른쪽 버튼으로 드래그하면, 놓인 위치까지 IK를 풀고
# 그 자세로 다시 계획·재생한다
PYTHONPATH=src env -u MUJOCO_GL python3 scripts/demo_plan_right_arm.py --interactive
```

목표를 지정하지 않으면 `--seed`로 시드된 rejection sampling으로 유효한
무작위 목표를 하나 뽑는다. `--start`/`--goal`에 `"q0 q1 ... q6"` 형태로 직접
7개 관절값을 줄 수도 있다. `--loop N`을 쓰면 한 목표에 도착할 때마다(``--execute``
없이도 계획만) 그 configuration을 다음 cycle의 시작점으로 삼아 새 무작위
목표를 다시 계획한다 — `--goal`은 첫 cycle에만 적용되고 이후는 항상
무작위다. `--show-tree`는 `--viewer`를 자동으로 켠다.

### 추가 장애물

기본적으로 빨간 구체 3개(`planning_obstacle_0..2`, 반지름 6cm)를 추가한다.
저장소의 `models/full_scene.xml`은 전혀 건드리지 않는다 — `_build_scene()`이
`mujoco.MjSpec.from_file`로 모델을 불러온 뒤 `spec.worldbody.add_geom(...)`으로
임시 지오메트리를 붙이고 그 자리에서 `spec.compile()`한다. 별도 `contype`/
`conaffinity`를 지정하지 않아 이 저장소의 기본값(오른팔 링크와 동일한
`contype=1 conaffinity=1`)을 그대로 물려받으므로 특별한 설정 없이도 충돌
검사에 잡힌다. 실제로 이 구체와 부딪히는 configuration이 있는지는
`checker.report(q)`의 `pair_name`에 `planning_obstacle_N`이 등장하는 것으로
직접 확인했다. 없이 비교하려면 `--no-obstacle`.

**위치를 고른 방법**: 처음엔 테이블 위 고정 기둥 하나였는데, 오른팔이
`RightArmSpace.sample()`(전체 관절 범위 균등 무작위)로 실제로 뻗는 손끝
위치를 5000개 표본으로 재보니 테이블 근처가 아니라 훨씬 넓고 다른 자리
(y가 테이블보다 훨씬 더 음수인 쪽)에 몰려 있어서 기둥이 거의 안 걸렸다.
그 표본 분포의 밀집 구간에서 후보 중심을 뽑고, `DEFAULT_START`가 계속
유효하게 남는 조합만 채택했다.

**크기를 고른 방법**: 반지름을 3cm→7cm까지 올려가며 (1) `DEFAULT_START`가
여전히 유효한지 (2) 무작위 유효 표본 비율이 너무 무너지지 않는지 (3) 시작점
기준 직선 경로 차단율이 적당한지를 실측했다. 6cm에서 시작 자세는 계속
유효(7cm부터 무효가 되기 시작)하고, 무작위 유효 표본 비율은 장애물
없을 때(58%) 대비 52%, 목표 50개 중 27개(54%)가 직선 경로 기준으로
막힌다 — 눈에 띄게 크면서도 계획이 항상 막히지는 않는 균형점이다.

### 트리·경로 시각화

`--show-tree`는 각 cycle의 계획이 끝나면(성공이든 실패든) `PlannerResult`의
`start_tree`/`goal_tree`(`TreeSnapshot`: 노드 배열 + 부모 인덱스 배열)를
MuJoCo 뷰어의 `user_scn`에 직접 그린다. 7차원 관절 공간 자체는 눈으로 볼 수
없으므로, 각 트리 노드(관절 configuration)를 오른손 site(`grasp_target_r`)의
world 좌표로 순전파(FK)해 3D 점 하나로 투영하고, 부모-자식 관계를 선분으로
잇는다. `mujoco.mjv_initGeom`으로 구체(노드)를, `mujoco.mjv_connector`로
선분(edge)을 `viewer.user_scn.geoms`에 채워 넣는 방식이며 씬 지오메트리
자체(`model`)는 건드리지 않는다. `--tree-pause-s`(기본 2.5초) 동안 정지
화면으로 보여준 뒤 지우고 실제 경로 재생으로 넘어간다.

`--execute`를 함께 쓰면 재생을 시작하기 직전에 최종 선택 경로(마젠타색 점+선,
`_draw_path`)를 그리고, 팔이 실제로 움직이는 동안에도 지우지 않는다 —
"이 경로를 따라가는 중"이라는 걸 눈으로 계속 비교할 수 있다. 재생이 끝나면
지우고 다음 cycle로 넘어간다. 경로 시각화는 트리와 같은 `_draw_trees`
헬퍼를 재사용한다 — 경로는 각 waypoint의 "부모"가 바로 앞 waypoint인
사슬형 트리와 구조적으로 같기 때문이다.

시작 쪽 트리는 초록(`#2a9e4a`), 목표 쪽 트리는 파랑(`#4f8ff2`), 경로는
마젠타(`#d94fa0`)다. 세 색 모두 `dataviz` 스킬의 OKLab Delta E 기반 CVD
(색각 이상) 검증을 통과한 조합이다 — 처음 썼던 초록·주황 조합은 protanopia
시뮬레이션에서 Delta E 2.8로 사실상 구분이 안 됐다(문턱 6, 목표 8). 아래
Q-space 페이지를 만들며 검증하다 발견해서 3D 뷰어 쪽 색도 함께 바꿨다.

### 마우스로 목표 옮기기 (`--interactive`)

`teleop_app.py`는 GLFW 마우스 콜백을 직접 짜서 목표 마커를 드래그하지만,
이 데모는 MuJoCo 뷰어에 이미 내장된 상호작용을 그대로 쓴다 — 씬에
`mocap="true"` body(`goal_marker`, 노란 구체, `contype=0 conaffinity=0`이라
충돌에 관여하지 않는다)를 하나 추가하면, 뷰어 기본 조작만으로(더블클릭으로
선택 → Ctrl+마우스 오른쪽 버튼 드래그) `data.mocap_pos`가 직접 갱신된다.
별도 마우스 이벤트 코드는 한 줄도 필요 없다.

`_run_interactive`는 이 위치를 ~30Hz로 폴링하다가, 한 지점에서 0.4초 이상
멈추면(`STABLE_HOLD_S`) 그 3D 점을 향해 IK를 푼다. IK는
`kinematics.joint_space.JointSpaceKinematics` 위에 짠 **position-only**
damped least-squares(`_ik_attempt`)이고, 여러 무작위 초기값에서 풀어
**수렴하면서 동시에 충돌도 없는** 첫 해를 채택한다(`_solve_valid_ik`) —
IK가 수렴만 하고 충돌하는 해는 버리고 다음 시드를 계속 시도한다. 해를
찾으면 현재 자세에서 그 관절 목표까지 RRT-Connect로 계획하고 재생한다.

**자세(orientation)는 일부러 목표에 안 건다.** 처음엔 세션 시작 시점의 손
자세를 계속 고정해 `kinematics.tasks.pose_error`로 위치+자세를 같이 풀었는데,
실제로는 도달 가능한 위치인데도 IK가 자주 수렴하지 않았다 — 예를 들어
`[-0.403, -0.865, 1.419]`는 자세를 고정하면 25번 재시도해도 못 풀지만
position-only로는 즉시 풀린다. 마커가 표현하는 정보는 3D 점 하나뿐이니,
7-DOF 중 위치 3개 자유도만 제약하고 나머지는 DLS 반복이 알아서 채우게
두는 게 맞다.

**남는 4개 여유 자유도는 무작위가 아니라 nullspace로 정칙화한다.** 위치
3개만 풀면 남는 자유도를 뭐로 채우는지가 자세를 얼마나 자연스럽게 만드는지
좌우한다. `_ik_attempt`는 표준적인 redundancy resolution 공식
`dq = J⁺e + (I − J⁺J)(k(q_ref − q))`을 쓴다 — 주 목표(위치 오차 줄이기)의
nullspace 안에서만, 부목표(현재 관절값 `q_ref`에 가깝게 유지)를 민다. 이
저장소의 반응형 `WholeBodyIK`가 쓰는 `regularization_task`와 같은 발상이다.
드래그한 지점 근처의 "제일 자연스러운" 자세가 장애물과 부딪히면, 정칙화를
걸어도 여전히 크게 재배치된 해가 나온다 — 그건 버그가 아니라 그 지점이
실제로 크게 돌아가야만 닿을 수 있는 자리라는 뜻이다(충돌 회피가 정칙화보다
항상 우선한다).

**같은 위치에 계속 머물러도 재계획을 반복하지 않는다.** "최근 몇 틱 동안
안 움직였는가"(`poll_ref`)와 "마지막으로 실제 계획을 실행한 위치"
(`processed_pos`)를 분리해서 추적한다 — 이 둘을 하나로 합치면, 사용자가
드래그를 멈춘 뒤에도 마커가 가만히 있는 것 자체가 "안정됨"으로 계속
재판정되어 0.4초마다 같은 목표를 무한히 재계획하는 버그가 생긴다(실제로
겪고 고쳤다). 새 위치는 마지막으로 처리한 위치에서 1cm 이상 떨어져 있을
때만 "새 목표"로 인정한다.

### Q-space(관절 공간) 시각화 — 참고용 익스포트

3D 뷰어의 트리 시각화는 각 관절 configuration을 손끝 site 하나의 좌표로
투영한 것이라, 7개 관절이 각각 어떻게 움직였는지는 보이지 않는다. 이를
보완하려고 한 번의 실제 계획 결과(`PlannerResult`의 `start_tree`/`goal_tree`/
`path`)를 JSON으로 내보내 **parallel coordinates** 인터랙티브 페이지로
그려 봤다 — 관절 7개를 세로 축 7개로 배치해 각 트리 노드·경로 waypoint를
축 7개를 가로지르는 선 하나로 그리고, 마우스를 올리면 관절 7개의 정확한
값(rad·deg)을 보여준다. **저장소에 체크인된 재사용 가능한 스크립트가 아니라
일회성으로 만든 결과물**이다 — 이런 뷰가 유용하다고 판단되면 `TreeSnapshot`
JSON 익스포트를 `scripts/`에 정식으로 추가하는 게 다음 단계다.

**실행(`--execute`) 재생의 한계**: waypoint마다 "수렴할 때까지 최대 3초 대기"
하는 임시 방식이다. 정식 시간 파라미터화(P2, `planning.trajectory`)가 관절
속도·가속도 상한을 지키는 매끄러운 궤적을 대신하게 된다 — 지금은 데모/디버그
용도다.

## 충돌 검사 계약

`ArmCollisionChecker`는 live 시뮬레이션 `MjData`를 **절대** 건드리지 않는다.
생성자에서 `copy.deepcopy(model)`로 전용 scratch 모델을 만들고, `set_snapshot(data)`가
호출될 때만 caller의 전체 qpos(캔 `can_free` free joint 포함)를 한 번 복사한다.
이후 `is_valid`/`report`/`clearance`는 이 스냅샷 위에서만 동작한다.

### 왜 `mj_forward`가 아니라 `mj_kinematics` + `mj_collision`인가

`mj_forward`는 동역학(`qfrc_bias`, 액추에이터, 제약 해)까지 계산하지만 유효성
판정에는 body pose와 접촉만 있으면 된다. `mj_kinematics`(pose) +
`mj_collision`(MuJoCo 자체 broad-phase)만 쓰면, 모델에 이미 선언된 `<exclude>`
(손가락 체인 등)와 `<pair>`(palm-table/floor 명시 margin 등) 규칙을 다시
구현하지 않고 그대로 물려받는다.

### 여유 거리(padding)와 CBF의 관계

`planning.collision.padding_m`(기본 0.012 m)만큼 계획 대상 geom의 `geom_margin`과
관련 `pair_margin` 행을 부풀려 판정한다. 값은 의도적으로
`whole_body_ik.collision_safe_distance_m`(0.01)과 `collision_buffer_m`(0.03)
사이에 둔다 — 계획한 경로 위에서는 국소 CBF가 활성화되지 않으면서도, 실행
중 예상 못 한 편차에는 CBF가 여전히 반응할 여유가 남는다
(`tests/test_planning_config.py::test_padding_is_between_cbf_safe_distance_and_buffer`가
이 부등식을 회귀 감시한다).

### 상자 가시성 가드

`models/full_scene.xml`의 `target_bin*` geom은 `class="target_bin_collision"`이라
raw 모델에서 `contype=2 conaffinity=0`으로 오른팔과 충돌하지 않는다. 오직
`imitation.simulation.environment.enable_task_collisions`가 이를 승격한다.
`ArmCollisionChecker(..., require_contact_geoms=(...))`는 생성 시점에 지정한
장애물 geom이 실제로 계획 대상과 충돌 가능한지 contype/conaffinity 비트마스크로
확인하고, 아니면 즉시 `ValueError`를 낸다 — 승격되지 않은 모델을 실수로 넘겨
상자를 관통하는 "안전한" 경로를 반환하는 사고를 막는다.

### 접촉 필터링 규칙

`mj_collision`이 반환한 접촉 중 다음은 무효 판정에서 제외한다:

1. 두 geom 모두 계획 대상(오른팔 링크·손·손가락) 밖이면 제외 — 바퀴-바닥
   접촉처럼 항상 존재하는 접촉이 판정을 오염시키지 않게 한다.
2. 두 geom이 모두 얼어붙은 손(hx5_r_base + finger_r_link1..20)에 속하면 제외
   — 손 자세는 계획 대상이 아니므로 손 내부 캡슐 겹침은 `q`와 무관한 상수다.
3. 호출자가 넘긴 `allowed_geom_pairs`(파지한 물체 예외 등)에 있으면 제외.

### 성능

`test_planning_collision.py::test_check_cost_is_bounded`가 측정한 실측치는
약 6700~7000 checks/s(≈ 150 µs/check)다. P1의 RRT-Connect 반복 상한은 이
예산에서 역산한다.

## clearance()가 캔을 보지 못하는 이유

`can_geom`이 붙은 body는 free joint(`can_free`)를 갖는데, 기존
`kinematics.tree.KinematicTree`의 FK/Jacobian은 scalar hinge/slide 체인만
지원한다(자유 관절을 지나는 body에 `point_jacobian`을 호출하면
`NotImplementedError`). 그래서 `obstacles.right_arm_collision_pairs`는 의도적으로
`can_geom`을 뺐다 — exact clearance 보고에서만 캔이 빠지고, boolean `is_valid`는
`mj_collision`의 broad-phase로 캔과의 충돌을 여전히 정확히 잡는다.

## 모바일 매니퓰레이터 P7 (베이스 + 오른팔)

로봇은 실제로 모바일이다 — `base_x`/`base_y`(slide)·`base_yaw`(hinge)
가상 관절이 있지만 직접 액추에이터는 없고, 3-모듈 스워브 드라이브(조향
6개 + 구동 6개)의 바퀴-지면 마찰로만 움직인다(`control.base.SwerveDrive`).
각 바퀴가 독립적으로 조향+구동하므로 **사실상 홀로노믹**이라 계획
관점에서 비홀로노믹(Reeds-Shepp류) 제약이 필요 없다.

반응형 whole-body IK(`control.whole_body.WholeBodyIK`)는 이미 베이스·리프트·
양팔을 하나의 weighted bounded differential IK로 묶어 풀지만, 손 목표
Cartesian pose에 매 제어 프레임 반응하는 로컬 솔버라 "베이스를 (x, y, yaw)로
보내라"는 지점-대-지점 주행이나 장애물을 우회하는 전역 경로는 만들지 못한다.
P7은 이 둘을 새로 만들지 않고, 그 사이에 빠진 **전역 베이스 배치 + 주행**만
채운다.

### 설계: decoupled 우선(Tier 1), 결합형(Tier 2)은 후속

실전에 배치된 모바일 매니퓰레이터(Fetch/TIAGo/HSR, PickNik/MoveIt Pro 등)의
기본 패턴은 **"베이스를 좋은 위치로 옮긴 뒤, 그 자리에서 고정-베이스 팔
계획기를 그대로 쓴다"**(decoupled)다 — 완전 결합 SE(2)×Rⁿ 샘플링 계획은
정적 베이스 자세로 안 풀리는 좁은 공간에서만 보조로 쓰는 게 일반적이다.
이 저장소도 이 패턴을 Tier 1로 먼저 구현했다 — 기존 팔 계획 파이프라인
(`RightArmSpace`/`ArmCollisionChecker`/`EdgeChecker`/`plan_rrt_connect`)을
**한 줄도 안 고치고** 재사용할 수 있기 때문이다. 결합형(Tier 2, 베이스+리프트+
팔을 하나의 11-DOF 공간으로 묶는 `WholeBodySpace`)은 P1의 플래너들이
이미 `space`/`checker` 추상 인터페이스에만 의존하도록 짜여 있어 인터페이스만
맞추면 나중에 언제든 얹을 수 있다 — 다만 검증할 실제 시나리오(좁은 통로 등)가
아직 없어 지금은 착수하지 않는다.

| Phase | 모듈 | 책임 |
|---|---|---|
| P7.0 | `reachability.build_reachability_map` | 베이스 프레임 기준 역-도달가능성 지도(IRM) 오프라인 구축 |
| P7.1 | `base_pose.select_base_pose` | 월드 목표에서 최선의 베이스 (x, y, yaw) 선택 |
| P7.1 | `base_pose.BaseFootprintChecker` | 베이스 발자국(2D) 충돌 검사 |
| P7.1 | `mobile_execution.drive_base_to_pose` | 선택된 베이스 자세까지 실제 주행(스워브 실행) |

### P7.0 — 역-도달가능성 지도(`reachability.py`)

베이스가 원점(`home` 키프레임)인 채로, 베이스 프레임 기준 (dx, dy, dz)
격자마다 "고정-베이스 오른팔 IK로 이 지점에 닿을 수 있는가"를 오프라인
한 번만 표로 만든다. 회전·이동에 불변이라 **로봇 1대당 한 번만 만들면
어떤 베이스 자세에서도 재사용**할 수 있다(월드→베이스 프레임 변환은
P7.1 쪽 책임).

```python
@dataclass(frozen=True)
class ReachabilityMap:
    grid_points: np.ndarray   # (N, 3) 베이스 프레임 상대 위치
    success_rate: np.ndarray  # (N,) 0.0/1.0 — 격자점별 단일 IK 시도 성공 여부

    def query(self, relative_xyz, *, k=8) -> float:
        ...  # 정확히 일치하면 그 점 값, 아니면 최근접 k점 역거리 가중 평균
```

IK는 새로 만들지 않는다 — `scripts/demo_plan_right_arm.py`의
`_ik_attempt`/`_solve_valid_ik`와 정확히 같은 계약(position-only DLS +
nullspace 정칙화, 여러 무작위 시드 중 수렴+무충돌인 첫 해 채택)을
`reachability._ik_attempt`/`reachability._probe`가 재사용한다.

`default_grid()`는 실측(3000개 무작위 유효 표본의 FK 손끝 위치 1~99
백분위: x∈[-0.70, 0.72], y∈[-1.02, 0.08], z∈[0.54, 1.89])에 여유를 둔
기본 경계상자를 `step=0.2`m 간격으로 채운다(504개 격자점). **`success_rate`는
이름과 달리 이산 0.0/1.0이다** — 여러 시드를 `_probe`가 이미 소진하므로
한 번의 빌드는 사실상 "그 seed에서 도달했는가"만 기록한다. 더 정밀한
연속값 통계(진짜 확률)가 필요하면 호출자가 여러 rng로 반복 호출해 평균을
내야 한다.

**실측**: 기본 격자(504점) 빌드에 81초 — 오프라인 1회성 비용으로 허용
범위. 오프라인 빌드이므로 온라인 계획 예산(P4 2분)엔 포함되지 않는다.

**테스트**(`tests/test_planning_reachability.py`, 4개): `query()`의 정확
일치·보간 로직은 순수 계산(MuJoCo 불필요, 빠름) + 작은 2점 격자로 실제
장면에서 "도달 가능 offset과 불가능 offset이 다르게 채점되는가"를 실제
IK로 검증하는 통합 테스트 1개.

### P7.1 — 베이스 자세 선택(`base_pose.py`)

```python
def select_base_pose(reachability_map, target_world_xyz, *, footprint_checker,
                      current_base_pose, candidate_radii, candidate_angles,
                      min_reachability=0.5) -> BasePoseResult:
    ...
```

목표 주위 (반경, 각도) 원 위에 후보 베이스 (x, y, yaw)를 만들고
① `reachability_map.query()` 점수 ② `footprint_checker` 무충돌 ③
`current_base_pose`와의 근접도 순으로 정렬해 최선을 고른다. 점수가 같으면
더 가까운 후보를 우선한다(불필요한 이동 회피). 후보가 하나도 기준을
만족 못 하면 `BasePoseResult(success=False, ...)`와 이유 문자열을 반환한다.

**yaw 후보도 `candidate_angles`를 그대로 재사용한다.** 로봇이 목표를 어느
축으로 "정면"으로 보는지 가정하지 않기 위해서다 — P7.0 실측 도달 영역이
+x보다 -y 쪽으로 훨씬 넓게 퍼져 있어(기본 격자 y∈[-1.1, 0.2]), 특정 축을
전제하면 틀리기 쉽다. 위치·방향 후보 조합 전체를 그냥 다 훑는다.

`world_to_base_frame(target_world_xyz, base_pose)`는 월드→베이스 SE(2)
변환이다 — `base_link`가 z로는 절대 움직이지 않으므로(높이는 `lift_joint`
담당) z는 그대로 통과시키고, xy는 `-yaw`만큼 회전한다:

```
relative_xy = R(-base_yaw) @ (target_world_xy - base_xy)
```

`BaseFootprintChecker`는 `ArmCollisionChecker`와 **같은 아키텍처를 그대로
따른다** — 생성자에서 `copy.deepcopy`한 scratch 모델, `set_snapshot(data)`가
호출될 때만 live qpos를 한 번 복사, `mj_kinematics`+`mj_collision`만 사용
(새 충돌 알고리즘을 만들지 않는다). `base_link`에 속한 `contype != 0`
geom만 감시 대상으로 잡아두므로 바퀴 geom(별도 body)과의 지면 접촉은
자동으로 제외된다. `padding_m`(기본 0.05m)만큼 그 geom들의 `geom_margin`을
부풀려 여유를 둔다.

**핵심 회귀 테스트**: `test_target_unreachable_from_far_base_becomes_reachable_after_repositioning`가
P7.0의 `build_reachability_map`을 **그대로 재사용**해(새 IK 검증 코드
없이) 먼 베이스 위치에서는 실제 IK로 도달 불가능(success_rate=0.0)했던
목표가 `select_base_pose`가 고른 위치에서는 도달 가능(success_rate=1.0)해짐을
실증한다. 이게 가능한 이유는 `build_reachability_map`이 실제로는 "베이스
원점 전용"이 아니라 grid point를 그냥 절대 world IK 타겟으로 쓰기
때문이다("베이스 원점" 요구사항은 결과 해석 쪽 약속일 뿐).

**테스트**(`tests/test_planning_base_pose.py`, 13개): SE(2) 변환의 독립
SO(2) 교차검증, stub 기반 `select_base_pose` 로직(최고점 선택·충돌
후보 배제·동점 시 거리로 tie-break·실패 사유), `BaseFootprintChecker`를
합성 벽 MJCF로 검증한 실제 충돌 판정 + padding 효과, 실제 장면에서
"home 자세는 유효"하다는 sanity check, 그리고 위 핵심 회귀 테스트.

**알려진 한계**: 실제 장면(`full_scene.xml` can-sort)엔 베이스 발자국
높이대([0.27, 0.51]m)에 겹치는 정적 장애물이 없다(테이블은 z∈[0.63,
0.73]로 그 위) — 그래서 진짜 충돌 판정은 합성 MJCF로만 검증했다. 낮은
장애물이 실제 장면에 추가되기 전까진 이 한계가 유효하다.

### 베이스 주행 실행(`mobile_execution.py`)

`select_base_pose`가 고른 목표 자세로 실제 이동하는 건 새 저수준 제어가
아니다 — `WholeBodyIK`는 손 목표 반응형이라 지점-대-지점 베이스 주행에
안 맞아서(설계 문서의 명시적 이탈), 기존 `control.base.SwerveDrive`를
목표-오차 루프로 얇게 감싼다:

```python
def drive_base_to_pose(model, data, target_pose, swerve_drive, *,
                        tolerance_m=0.03, tolerance_rad=0.02,
                        kp_linear=1.5, kp_angular=1.5,
                        max_speed=0.6, max_steps=20000) -> BaseTransitReport:
    ...
```

매 스텝: 월드 프레임 위치 오차 → 비례 게인 → 현재 yaw로 `R(-yaw)` 회전해
차체(body) 프레임 twist `(vx, vy, wz)` → `swerve_drive.update_twist(twist, dt)`
→ 반환된 바퀴별 (조향각, 구동속도)를 `data.ctrl`에 직접 써서 `mj_step`.
새 액추에이터·새 기구학 없음 — 오케스트레이션 코드만 새로 추가됐다.

**테스트 커버리지의 한계(정직한 고지)**: 실 장면·실 `SwerveDrive`로 0.15m
변위 하나에 대한 smoke test만 있다 — P7.1에서 가장 검증이 얕은 부분이다.

### 베이스 배치가 필요한 상황 확인해 보기

```bash
# reachability map: 순수 로직(격자 보간)만 — MuJoCo 없이 빠르게
MUJOCO_GL=osmesa .venv/bin/python -m pytest -q tests/test_planning_reachability.py

# base_pose 전체(로직 + 실제 장면 충돌·회귀 테스트)
MUJOCO_GL=osmesa .venv/bin/python -m pytest -q tests/test_planning_base_pose.py
```

엔드투엔드 데모(`scripts/demo_plan_mobile_manipulator.py`, "베이스를
옮긴 뒤 기존 팔 파이프라인으로 계획·실행")는 아직 없다 — P7.1까지는
라이브러리 레이어(reachability map + 베이스 자세 선택 + 주행 실행)만
완성됐고, 이를 엮는 통합 데모 스크립트는 다음 단계다.

## P7 Tier 2 타당성 평가 (설계 문서 — 미착수)

**이 절은 결합형(coupled) whole-body 플래닝이 기술적으로 가능한지 코드
근거로 판단한 타당성 평가 + 청사진이다. 아래 어떤 코드도 아직 구현되지
않았다** — Tier 1(위 절)이 실전에서 검증된 뒤, 정적 베이스 배치로 안
풀리는 시나리오가 실측으로 확인되면 착수한다는 원칙은 그대로 유효하다.

### 결론: 가능하다 — 가장 어려운 부분이 이미 검증돼 있다

Tier 2가 필요로 하는 건 base_x·base_y·base_yaw·lift_joint·오른팔
7관절을 하나로 묶은 11-DOF 공간에 대한 FK/Jacobian, 충돌 검사, 그리고
그 공간에서 동작하는 샘플링 플래너다. 조사 결과 **이 중 FK/Jacobian은
이미 프로덕션에서 쓰이고 있었다** — `control.whole_body.WholeBodyIK`가
지금도 `self.kinematic_tree = KinematicTree(model)`
(`control/whole_body.py:194`) 하나로 `base_x, base_y, base_yaw, lift_joint`
+ 양팔(`control/whole_body.py:163-176`의 `self.joint_names`/`self.joint_ids`
조립)을 묶은 Jacobian을 매 제어 프레임 `self.kinematic_tree.forward_site(
qpos, site_id, self.joint_ids)`(`control/whole_body.py:777-789`)로 계산한다.
`kinematics.tree.KinematicTree`는 스칼라 hinge/slide 관절만 지원한다는
제약이 있지만(자유/볼 관절은 `NotImplementedError`), `base_x`/`base_y`
(slide)·`base_yaw`(hinge)는 `models/full_scene.xml:312-315`에서 보듯
`base_link` 위에 순서대로 쌓인 평범한 스칼라 관절 3개일 뿐이다 — "가상
평면 관절이라 특별 취급이 필요하다"는 우려는 사실이 아니었다.

`planning/`의 알고리즘도 다시 확인했다 — `rrt_connect.py`, `shortcut.py`,
`trajectory.py`, `local_path.py`의 `EdgeChecker`는 전부 `space`/`checker`/
`edge_checker` 추상 인터페이스에만 의존하고 팔 전용 하드코딩이 없다
(docstring의 "7-DOF" 언급은 설명용 주석일 뿐 동작에 영향 없음). **이
네 파일은 한 줄도 안 고치고 재사용 가능**하다 — Tier 1 설계 때 이미
확인한 "좋은 추상화는 미래 확장을 값싸게 만든다"는 통찰이 다시 한번
맞아떨어졌다.

### 그래도 필요한 세 가지 수정/설계 결정

1. **`ArmCollisionChecker`의 자기충돌 판정이 팔 전용 접두어로 하드코딩돼
   있다.** `_collect_planned_geoms`(`planning/collision_state.py:84-94`)가
   `prefixes = ("arm_r_link",) + _FROZEN_HAND_BODY_PREFIXES`
   (`_FROZEN_HAND_BODY_PREFIXES = ("hx5_r_base", "finger_r_link")`, 26번째
   줄)로 "계획 대상 geom" 목록을 만든다 — 베이스·리프트 body는 이 목록에
   없으므로, `WholeBodySpace`를 그냥 주입해도 **베이스·리프트의 충돌이
   조용히 무시된다**(`is_valid()`가 위험한 자세도 항상 통과시킬 수 있음).
   생성자에 `planned_body_prefixes` 파라미터를 추가해(기본값 = 현재
   오른팔 전용 튜플, 하위호환 100% 유지) `_collect_planned_geoms`가 이를
   받아 필터링하도록 고쳐야 한다 — 새 클래스(`WholeBodyCollisionChecker`)는
   필요 없다. `space` 자체가 이미 생성자 주입식이라
   (`self.space = space if space is not None else RightArmSpace.from_model(...)`,
   `planning/collision_state.py:61`) `ArmCollisionChecker(model,
   whole_body_space, planned_body_prefixes=("base_link", "lift_link",
   "arm_base_link", "arm_r_link", "hx5_r_base", "finger_r_link"))`처럼
   호출하면 스냅샷·`is_valid`/`report`/`clearance` 패턴은 전부 그대로
   재사용된다. 왼팔·헤드·손가락은 여전히 스냅샷에 고정된 장애물로
   남는다(설계 범위는 베이스+리프트+오른팔, 왼팔 협조는 범위 밖).
2. **`trajectory.time_parameterize`는 모든 차원에 같은 스칼라 속도·
   가속도 상한을 쓴다**(모듈 docstring이 명시: "이 저장소는 모든 관절에
   같은 스칼라 속도·가속도 상한을 쓰므로") — base_x/base_y(m)와 관절(rad)이
   섞인 11-DOF에 그대로 쓰면 어느 한쪽이 다른 쪽 속도를 억지로 지배하게
   돼 물리적으로 말이 안 된다.
3. **`base_x`/`base_y`/`base_yaw`는 XML에 `range`가 없다**(무제한 — 위
   313-315번째 줄 참고). `floor` geom도 `size="0 0 0.05"`인 무한 평면이라
   (`models/full_scene.xml:310`) 샘플링 경계가 XML 어디에도 없다.
   `RightArmSpace.sample()`의 관례를 그대로 물려받으면 문제가 된다 —
   그 함수는 "범위가 없는 자유도(현재 없음)는 uniform이 정의되지 않으므로
   0으로 둔다"(`planning/arm_state.py:94`, 실제로 지금까지는 이 분기를
   타는 관절이 하나도 없었다)고 명시돼 있다 — `WholeBodySpace`가 이
   관례를 그대로 쓰면 **베이스가 항상 원점에 고정되는 조용한 버그**가
   된다. base_x/base_y는 독자적인 bounded box(생성자 인자, 기본값은
   현재 베이스 위치 주변 반경 N m), base_yaw는 `[-π, π)` 균등 샘플이
   필요하다.

### 결정적 사실: 지금 이 씬엔 결합 계획이 필요한 시나리오가 없다

`models/full_scene.xml`의 정적 장애물(`table`: z∈[0.6316, 0.7316],
`target_bin`/`target_bin_red`: z≈0.63~0.82)은 전부 베이스 충돌 박스의
z-범위(`base_link` 자식의 `<geom type="box" size="0.2 0.2 0.12" pos="0 0
0.24">`, world z∈[0.27, 0.51], `models/full_scene.xml:321`) 위에 떠 있다
— 베이스 발자국과 절대 안 겹친다. 즉 **decoupled(Tier 1)가 못 푸는 진짜
상황이 현재 씬엔 존재하지 않는다.** 이건 Tier 2를 막는 요인이 아니라
"지금 당장 급하지 않다"는 근거다 — 착수 시점엔 `test_planning_base_pose.py`의
`_WALL_MJCF`처럼 합성 MJCF로 "베이스만 옮기거나 팔만 움직여선 못 풀고
둘을 같이 움직여야 풀리는" 최소 시나리오(예: 낮은 파티션 + 그 너머의
목표)를 새로 만들어야 한다 — 실제 씬에 그런 장애물을 추가하기 전까진
Tier 2를 검증할 방법이 없다.

### 청사진 (착수 시 따를 설계, 지금은 작성만)

- **`WholeBodySpace`**(신규, `planning/whole_body_space.py`) —
  `RightArmSpace`와 같은 공개 인터페이스(`sample`, `distance`,
  `max_component`, `interpolate`, `steer`, `contains`, `write`, `.n`,
  `.joint_names`, `.joint_ids`, `.lower`, `.upper`, `.limited`)를 11-DOF로
  구현한다. 위치(m)와 각도(rad)를 동차로 만드는 per-차원 스케일 벡터를
  두고 `distance`(가중 L2)·`max_component`(가중 L∞ — `EdgeChecker.steps`/
  `time_parameterize`가 실제로 쓰는 함수)·`steer`가 이 스케일을 일관되게
  적용한다. `interpolate`는 스케일과 무관하게 선형(base_yaw만 최단각
  wraparound).
- **`ArmCollisionChecker`에 `planned_body_prefixes` 파라미터 추가** — 위
  1번 항목대로, 기존 파일에 대한 작은 하위호환 확장.
- **실행(계획 후 재생)은 이번 설계 범위 밖으로 명시적으로 보류한다.**
  Tier 1과 달리 Tier 2의 경로는 베이스와 팔이 동시에 움직여야 하는데,
  `mobile_execution.drive_base_to_pose`를 waypoint마다 반복 호출하는
  방식은 매번 정지할 때까지 수렴한 뒤에야 다음 목표로 넘어가는
  stop-start 방식이라 "결합된 이동"의 취지와 다르다. 진짜 동시 실행은
  매 물리 스텝마다 베이스 트위스트와 팔 관절 명령을 함께 계산하는 새
  루프가 필요하고, 이는 P3 실행 모듈(PR #8, 아직 미병합)에도 의존한다 —
  Tier 2가 실제 착수될 때의 후속 과제로 남긴다.
- **1차 목표 알고리즘은 `plan_rrt_connect`로 한정한다.** `rrt_star.py`/
  `chomp.py`는 이 저장소 `main`에 없다(각각 PR #4/#5, 미병합·미리베이스
  브랜치에만 존재 — `chomp` 브랜치는 `settings.py`/`config/default.yaml`의
  최근 변경 이전 시점에서 갈라져 그대로 병합하면 회귀가 남는다). RRT*/
  CHOMP whole-body 지원은 그 PR들이 병합·리베이스된 뒤의 별도 후속 과제다.

## 다음 단계

P2(shortcut 평활화 + 시간 파라미터화)부터는 자동 연구 루프(`docs/agents.md`)가
`TODO.md`를 보고 진행한다. P7 Tier 2(결합형 `WholeBodySpace`)는 위 타당성
평가에서 "가능하다"고 결론 났지만, 검증할 실제 시나리오(합성 MJCF 포함)가
먼저 준비돼야 착수 의미가 있다 — 사용자의 우선순위 결정을 기다린다.
로드맵 전체는 [`docs/prd.md`](../prd.md) §2를 참고.
