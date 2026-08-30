# 오른팔 모션 플래닝

`src/ffw_sh5_grasp/planning/`은 오른팔 7-DOF(`arm_r_joint1..7`)를 위한
sampling-based 모션 플래너다. 베이스·리프트·헤드·손가락·왼팔은 계획 대상이
아니라 질의 시점의 상태로 고정한 장애물로 취급한다.

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

## 다음 단계

P2(shortcut 평활화 + 시간 파라미터화)부터는 자동 연구 루프(`docs/agents.md`)가
`TODO.md`를 보고 진행한다. 로드맵 전체는 [`docs/prd.md`](../prd.md) §2를 참고.
