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

## 현재 구현 (P0 + P1 + P5 대안 플래너)

| 모듈 | 책임 |
|---|---|
| `arm_state.RightArmSpace` | 오른팔 관절 이름·id·qpos 주소·범위. 샘플링·보간·클리핑 |
| `obstacles.right_arm_collision_pairs` | `clearance()`의 정확한 거리 보고에 쓰는 충돌 쌍 목록 |
| `collision_state.ArmCollisionChecker` | boolean `is_valid(q)` + exact `clearance(q)` |
| `local_path.EdgeChecker` | 두 configuration 사이 선분의 충돌 검사(이분 순서) |
| `settings.load_collision_settings` | `config/default.yaml`의 `planning.collision.*` 로더 |
| `rrt_connect.plan_rrt_connect` | 두 트리 EXTEND/CONNECT 표준 RRT-Connect. 결정론적 seed. 반환값에 `TreeSnapshot`(탐색한 전체 트리)도 포함 |
| `rrt_star.plan_rrt_star` | 단일 트리 RRT*. 시간 예산이 끝날 때까지 계속 탐색하며 비용(경로 길이)을 개선 |

평활화·시간 파라미터화(P2)와 정식 실행 모듈(P3)은 아직 없다.

### RRT-Connect 알고리즘 요약

두 트리(시작 쪽, 목표 쪽)를 번갈아 확장한다. 매 반복:

1. `goal_bias` 확률로 목표를, 아니면 `RightArmSpace.sample()`로 관절 범위 안
   균등 무작위 표본을 하나 뽑는다.
2. 현재 트리(`tree_a`)를 그 표본 방향으로 `EXTEND`(최근접 노드에서
   `step_size_rad`만큼 스티어 + `EdgeChecker`로 유효성 확인, 유효하면 노드 추가).
3. `EXTEND`가 트인 방향으로 나아갔다면(`ADVANCED`), 반대 트리(`tree_b`)를
   그 새 노드 쪽으로 `CONNECT`(닿거나 막힐 때까지 `EXTEND`를 반복)한다.
4. `CONNECT`가 닿으면(`REACHED`) 두 트리의 root-to-node 경로를 이어 붙여
   성공. 아니면 `tree_a`/`tree_b`를 swap하고 다음 반복.

`time_budget_s`가 실제 종료 조건이고 `max_iterations`는 폭주 방지용 상한이다.
`_Tree.nearest`는 KD-tree가 아니라 선형 탐색이다 — 7-D·수천 노드 규모에서는
의존성 없이도 충분히 빠르다(§성능 참고). **첫 해를 찾으면 즉시 반환**하므로
경로 품질(길이)이 운에 좌우된다 — 아래 RRT*가 이 문제를 다룬다.

### RRT* 대안 플래너 (P5)

`rrt_star.plan_rrt_star`는 시간 예산이 끝날 때까지 계속 탐색하며, 새 노드를
삽입할 때마다 (1) 반경(`rewire_radius_rad`) 안 근접 노드 중 비용(root부터
누적 거리)이 가장 낮은 쪽을 부모로 선택하고 (2) 반경 안 다른 노드들도 새
노드를 거치는 편이 더 싸면 부모를 새 노드로 바꾸는(rewire) 방식으로
점진적으로 경로 비용을 개선한다.

**설계 선택 — 왜 단일 트리인가.** RRT-Connect의 두 트리 bidirectional
connect와 rewiring을 결합하면 복잡도가 크게 늘어난다(양쪽 트리 모두
rewiring을 지원해야 하고, 두 트리가 만나는 지점의 비용 계산도 더 복잡해진다).
그래서 이 모듈은 의도적으로 표준 단일 트리 RRT*로 설계했다.

**설계 선택 — 왜 고정 rewiring 반경인가.** 점근적 최적성을 위한 이론적
shrinking radius(`gamma * (log n / n)^(1/d)`)는 7-DOF 관절 공간에서 반경이
매우 빠르게 줄어들어 rewiring이 사실상 거의 일어나지 않는다. 대신 고정
반경(`rewire_radius_rad`, CLI 기본값 `2 * step_size_rad`)을 쓴다 —
`trajectory.py`가 코너 blending을 범위 밖으로 명시 보류한 것과 같은 종류의
실용적 단순화다. 점근적 최적성을 엄밀히 보장하지는 않지만, 시간 예산 안에서
비용을 계속 개선한다는 핵심 이득은 그대로 유지된다
(`tests/test_planning_rrt_star.py::test_rewiring_improves_cost_over_time`이
이 성질을 회귀 감시한다).

**설계 선택 — informed sampling은 거부 표집으로.** 첫 해를 찾은 뒤부터는
`dist(start, x) + dist(x, goal) < best_cost`인 표본만 받아들이는 거부
표집을 쓴다(`informed_sampling=True`가 기본). 7-D 타원체의 회전행렬을 직접
구성하는 정식 Informed RRT* 샘플러보다 훨씬 단순하면서, 이미 찾은 해보다
더 나은 해가 지날 수 있는 영역으로 탐색을 좁히는 같은 효과를 낸다.

**실측으로 발견한 튜닝 — RRT-Connect의 기본값을 그대로 물려주면 안 된다.**
RRT-Connect는 두 트리가 서로를 향해 CONNECT하며 한 반복에 여러 스텝을
한꺼번에 전진할 수 있어, `goal_bias=0.1`만으로도 이 저장소의 실제
can-sort 장면에서 보통 1회 반복 안에 성공한다. RRT*는 단일 트리라 한
반복에 `step_size_rad`만큼만 전진하는데, 실측 결과 같은 `goal_bias=0.1`·
목표 연결 반경(`goal_tolerance_rad`)을 `step_size_rad`로 두면 이 장면에서
40초·6750회 반복 안에도 목표에 못 닿는 경우가 있었다(단일 트리 RRT류가
bidirectional 탐색보다 점-대-점 질의에 훨씬 느리다는 건 RRT-Connect가
애초에 고안된 이유이기도 한 잘 알려진 트레이드오프다). `goal_bias=0.3`,
`goal_tolerance_rad=0.5`로 올리자 같은 예산 안에서 안정적으로 성공했다 —
데모 CLI는 `--planner rrt_star`일 때 이 값들과 더 넉넉한 기본
`--time-budget-s`(30초, RRT-Connect는 5초)를 자동으로 쓴다. 그래도
실제 장면에서 성공까지 보통 수천 회 반복·10~20초가 걸린다 — RRT-Connect의
수 밀리초 대비 훨씬 느리다. 이건 **비용 최적화**와 **빠른 첫 해**라는
서로 다른 목표를 고른 트레이드오프이지 버그가 아니다.

```bash
# RRT*로 계획 + 재생 (RRT-Connect보다 훨씬 오래 걸리지만 더 짧은 경로를 낸다)
PYTHONPATH=src MUJOCO_GL=osmesa python3 scripts/demo_plan_right_arm.py \
    --seed 3 --execute --planner rrt_star

# 두 플래너의 경로 길이를 직접 비교
PYTHONPATH=src MUJOCO_GL=osmesa python3 scripts/demo_plan_right_arm.py --seed 3 --execute
PYTHONPATH=src MUJOCO_GL=osmesa python3 scripts/demo_plan_right_arm.py --seed 3 --execute --planner rrt_star
```

`--show-tree`로 뷰어에 그리면 RRT*는 단일 트리이므로 목표 쪽(파랑)에는
목표점 하나만 표시된다(시작 쪽 초록 트리만 실제로 자란다) — 트리 시각화
코드 자체는 두 플래너가 같은 `PlannerResult`/`TreeSnapshot` 인터페이스를
공유해서 고칠 필요가 없었다.

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

## 다음 단계

RRT*(P5) 초안(`planning/rrt_star.py`)까지 구현·데모 연결이 끝났다. 다음은
`MP-0017`(RRT-Connect vs RRT* 50-seed 정식 비교표, `RESULTS.md`) — 벤치마크
하네스(`MP-0013`)가 먼저 있어야 한다. P2(shortcut 평활화 + 시간
파라미터화)와 P3(정식 실행 모듈)는 자동 연구 루프(`docs/agents.md`)가
`TODO.md`를 보고 진행한다. 로드맵 전체는 [`docs/prd.md`](../prd.md) §2를 참고.
