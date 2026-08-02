# 기구학 API

이 페이지의 함수는 목표 정책이나 actuator를 다루지 않는다. 모델 구조와 관절 상태를
받아 pose, Jacobian, IK 해, 충돌 거리 미분을 계산한다. Quaternion은 MuJoCo 순서인
`(w, x, y, z)`, Jacobian은 위 3행이 선속도이고 아래 3행이 각속도다.

## 회전 유틸리티 `kinematics.rotations`

| 함수 | 직관적인 기능 | 입력 | 반환 |
|---|---|---|---|
| `normalize_quaternion(quaternion)` | 길이와 부호가 제각각인 quaternion을 안정적인 대표값으로 만든다 | 4-vector | 단위 `wxyz`; 잘못된 0-norm은 identity |
| `shortest_orientation_error(target_quaternion, current_quaternion)` | 현재 자세에서 목표 자세로 가는 가장 짧은 회전 화살표를 구한다 | target/current `wxyz` | world-frame axis-angle 3-vector, rad |
| `rotation_from_quaternion(quaternion)` | quaternion을 점·축 계산용 회전행렬로 바꾼다 | `wxyz` | (3\times3) 행렬 |
| `quaternion_from_rotation(rotation)` | 회전행렬을 정규화된 MuJoCo quaternion으로 바꾼다 | (3\times3) 행렬 | 단위 `wxyz` |
| `axis_rotation(axis, angle)` | 한 축 주위의 회전을 Rodrigues 식으로 만든다 | 3D 축, rad | (3\times3) 행렬 |
| `clip_norm(vector, limit)` | 방향은 유지하고 벡터 길이만 제한한다 | 벡터, 음이 아닌 limit | 제한된 벡터 복사본 |
| `wrap_angle(angle)` | 누적된 각도를 비교 가능한 한 바퀴 범위로 접는다 | rad | `[-π, π)` rad |
| `skew(vector)` | 외적 `v × x`를 행렬 곱으로 표현한다 | 3-vector | (3\times3) skew 행렬 |

자세 오차의 곱셈 순서와 부호 선택은
[Quaternion과 자세 오차](../guide/quaternion-math.md)에 수식으로 설명한다.

## 결과 자료형

### `SiteKinematics`

`KinematicTree.forward_site()`와 `WholeBodyIK.site_state()`가 반환하는 한 site의 계산
결과다.

| 필드 | 의미 |
|---|---|
| `position` | world 위치, shape `(3,)`, m |
| `quaternion` | world 자세, shape `(4,)`, `wxyz` |
| `jacobian` | 선택한 관절에 대한 geometric Jacobian, shape `(6, N)` |
| `joint_frames` | 충돌 point Jacobian에 재사용할 관절별 world 축·anchor cache |

`KinematicJoint`, `KinematicBody`, `KinematicSite`는 MuJoCo model에서 복사한 불변 구조
레코드다. 각각 관절 축/범위, body 고정변환/부모, site 고정변환을 보관한다.

## `KinematicTree(model)`

**직관:** MuJoCo model을 매번 조회하지 않도록, root에서 원하는 body/site까지 직접
따라갈 수 있는 작은 기구학 지도를 한 번 만든다.

- **입력:** 컴파일된 `mujoco.MjModel`.
- **보관:** body/joint/site의 불변 정보, 이름 lookup, 기본 `qpos0`.
- **사용 시점:** 같은 모델에서 FK·Jacobian·충돌 미분을 반복 계산할 때 한 번 생성한다.

### `KinematicTree.forward_site(qpos, site_id, joint_ids)`

- **직관:** root부터 site까지 관절 변환을 누적해 “손이 지금 어디 있고, 각 관절을
  움직이면 어느 방향으로 갈지”를 함께 계산한다.
- **입력:** 모델 전체 형태의 `qpos`, MuJoCo site id, Jacobian 열 순서를 정할 joint id 목록.
- **반환:** `SiteKinematics`. 모든 pose와 Jacobian은 world frame이다.
- **부작용:** 없음. 전달된 `qpos`나 live `MjData`를 수정하지 않는다.

### `KinematicTree.point_jacobian(qpos, body_id, point_world, joint_ids, frame_cache=None)`

- **직관:** body 원점이 아닌 충돌 최근접점 하나가 관절 속도에 따라 어떻게 움직이는지
  계산한다.
- **입력:** 전체 `qpos`, 점이 붙은 body id, 현재 world 점, 열 순서의 joint id,
  선택적 관절 frame cache.
- **반환:** 점의 world 선속도 Jacobian, shape `(3, N)`.
- **사용 시점:** geometry distance gradient처럼 body 위 특정 점의 속도가 필요할 때.

Tree 생성 이유와 FK 연결은 [Kinematic Tree](../guide/kinematic-tree.md), 열의 유도는
[FK와 Geometric Jacobian](../guide/forward-kinematics.md)에 있다.

## `KinematicsSolver`

이름 기반 관절 벡터를 `KinematicTree`의 모델 전체 `qpos`에 끼워 넣고, 단일 site
FK와 반복 DLS IK를 제공하는 편의 계층이다.

### `KinematicsSolver.from_mjcf(path, site_name, joint_names, **kwargs)`

- **직관:** XML 경로만 알고 있을 때 model 로드부터 solver 생성까지 한 번에 한다.
- **입력:** MJCF 경로, 끝단 site 이름, 순서가 고정된 관절 이름 목록, 선택적 damping 등.
- **반환:** 초기화된 `KinematicsSolver`.
- **오류:** site/joint 누락 또는 지원하지 않는 관절 구성.

### `KinematicsSolver.forward(q, context_qpos=None)`

- **직관:** 선택한 팔 관절각을 넣어 손의 world pose와 Jacobian을 바로 구한다.
- **입력:** solver 관절 순서의 `q`; 선택적 모델 전체 `context_qpos`는 base·lift·반대팔
  같은 나머지 관절 상태를 제공한다.
- **반환:** `SiteKinematics`.
- **주의:** `context_qpos`도 복사하므로 호출자 배열과 live 물리 상태를 바꾸지 않는다.

### `KinematicsSolver.forward_kinematics(q, context_qpos=None)`

- **직관:** Jacobian이 필요 없는 기존 코드가 손 pose 두 값만 받는 호환 이름이다.
- **반환:** `(world_position, world_quaternion)`.
- **권장:** 새 코드에서 Jacobian도 쓸 가능성이 있으면 `forward()`를 사용한다.

### `KinematicsSolver.solve_pose(...)`

```python
solve_pose(
    q_init, target_pos, target_quat,
    max_iter=..., pos_tol=..., ori_tol=..., ori_weight=...,
    context_qpos=None,
)
```

- **직관:** 한 초기 관절 자세에서 시작해 위치를 우선하고 자세를 그다음으로 줄이는
  DLS step을 반복한다.
- **입력:** 초기 관절각, world 목표 위치·quaternion, 반복/오차 한계, 자세 가중치,
  선택적 전체 자세 문맥.
- **반환:** `(q_solution, position_error_norm, orientation_error_norm)`.
- **종료:** 두 오차가 tolerance 안이거나 `max_iter`에 도달했을 때.
- **사용 시점:** 단일 팔의 연속 target처럼 이전 해가 좋은 초기값일 때.

### `KinematicsSolver.solve_pose_multistart(...)`

- **직관:** 첫 초기값이 나쁜 국소해에 걸렸을 때 여러 시작 자세로 다시 풀어 가장 좋은
  결과를 고른다.
- **입력:** `solve_pose` 입력에 난수 생성기 `rng`, 재시작 횟수와 성공 tolerance가 추가된다.
- **반환:** `(q_solution, position_error_norm, orientation_error_norm, success)`.
- **사용 시점:** offline 목표 생성이나 큰 pose 점프. 매 frame 실시간 루프에서는 계산
  예산을 먼저 확인한다.

`InverseKinematics`는 `kinematics.legacy`에 남긴 기존 import용 `KinematicsSolver`
별칭이다.
새 코드에서는 `KinematicsSolver`를 직접 사용한다. DLS 전개는
[DLS와 위치 우선 IK](../guide/ik-math.md), 반복 절차는 [단일 팔 IK](../guide/ik.md)에 있다.

## 충돌 기구학 `kinematics.collision`

### `CollisionPair`

감시할 두 geom과 거리 계산 mode를 정의한다. 일반 geometry pair 외에 table-top과
bounding-sphere 근사를 표현할 수 있다.

### `CollisionConstraint`

활성 거리 계산 한 건의 결과다. pair의 `name`, `distance`, 전체 joint 열에 대한
`gradient`, 두 최근접점 `point_a/point_b`를 보관한다.

### `default_collision_pairs(model)`

- **직관:** 로봇 self/table/can 중 CBF가 감시해야 할 쌍만 모델 이름으로 구성한다.
- **입력:** `MjModel`.
- **반환:** `CollisionPair` tuple.
- **주의:** 손가락–물체와 wheel–floor처럼 의도된 접촉은 제외한다.

### `collision_distance_gradient(...)`

```python
collision_distance_gradient(
    model, data, pair, tree, joint_ids,
    max_distance, frame_cache=None,
)
```

- **직관:** 두 형상이 얼마나 떨어졌는지와 각 관절을 어느 방향으로 움직여야 거리가
  늘어나는지를 동시에 계산한다.
- **입력:** 현재 model/data, 감시 pair, 공유 tree, WBIK joint 열 순서, 계산할 최대 거리,
  선택적 frame cache.
- **반환:** 거리 안이면 `CollisionConstraint`, 멀리 있거나 유효하지 않으면 `None`.
- **사용 시점:** WBIK CBF 부등식을 만들거나 동일한 활성 충돌을 시각화할 때.

최근접점 Jacobian에서 \(\dot d=\nabla d\dot q\)까지의 유도는
[Collision distance와 gradient](../guide/collision-kinematics.md)에 있다.

## MuJoCo 보조 함수

### `find_actuator_for_joint(model, joint_id)`

- **직관:** 한 관절을 실제로 구동하는 actuator가 있는지 안전하게 찾는다.
- **입력:** `MjModel`, joint id.
- **반환:** actuator id 또는 연결된 actuator가 없으면 `None`.
- **주의:** 반환값을 검사하지 않고 `data.ctrl[None]`에 쓰면 NumPy broadcasting으로
  전체 control 배열이 바뀔 수 있다.
