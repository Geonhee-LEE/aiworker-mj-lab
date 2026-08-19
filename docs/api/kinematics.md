# 기구학 API

`ffw_sh5_grasp.kinematics`는 회전 계산, FK·Jacobian, velocity task와 제약, 수치 IK,
충돌 거리를 제공한다. 별도 표기가 없으면 입력 배열을 변경하지 않는다.

## 회전 계산 { #rotations }

모듈은 `kinematics.rotations`이며 quaternion 순서는 `(w, x, y, z)`다.

| 함수 | 반환 |
|---|---|
| `normalize_quaternion(q)` | 정규화하고 `w >= 0`으로 맞춘 quaternion; 무효 입력은 단위 quaternion |
| `multiply_quaternions(*quaternions)` | 왼쪽부터 곱한 quaternion |
| `inverse_quaternion(q)` | 단위 quaternion의 역회전 |
| `rpy_deg_to_quat(rpy_deg)` | degree RPY → quaternion |
| `quat_to_rpy_deg(q)` | quaternion → degree RPY list |
| `shortest_orientation_error(target, current)` | world-frame 최단 회전벡터, shape `(3,)` |
| `rotation_from_quaternion(q)` | 3×3 회전행렬 |
| `quaternion_from_rotation(R)` | 정규화된 quaternion |
| `axis_rotation(axis, angle)` | Rodrigues 3×3 회전행렬 |
| `clip_norm(vector, limit)` | 방향을 유지하며 norm을 제한한 벡터 |
| `wrap_angle(angle)` | `[-π, π)` 범위의 각도 |
| `skew(vector)` | 3×3 skew-symmetric 행렬 |

## Pose와 velocity task { #tasks }

모듈은 `kinematics.tasks`다.

| 자료형 | 필드 |
|---|---|
| `PoseError` | `position`, `orientation`; `position_norm`, `orientation_norm` |
| `VelocityTask` | `name`, 정규화된 `matrix`, `target` |

| 함수 | 동작·반환 |
|---|---|
| `pose_error(current_pos, current_quat, target_pos, target_quat)` | world-frame `PoseError` |
| `pose_velocity_command(error, **gains_and_limits)` | `[linear(3), angular(3)]` 목표 twist |
| `normalized_weights(strengths, velocity_scales)` | `strength / scale²` 배열 |
| `velocity_task(name, jacobian, target_velocity, strengths, velocity_scales)` | `VelocityTask` |
| `regularization_task(name, target_velocity, strengths, velocity_limits)` | identity Jacobian의 `VelocityTask` |
| `stack_velocity_tasks(tasks, variable_count)` | `(stacked_matrix, stacked_target)` |

`pose_velocity_command()`의 필수 keyword는 `position_gain`, `orientation_gain`이다.
현재 twist, 선·각속도 damping과 속도 상한은 선택 항목이다.

## 속도·위치 제약 { #constraints }

모듈은 `kinematics.constraints`다.

| API | 동작·반환 |
|---|---|
| `VelocityBarrier(name, distance, gradient, lower)` | `gradient @ qdot >= lower` 한 행 |
| `joint_velocity_bounds(current, limits, limited, ranges, dt, *, margin, gain)` | 물리 속도와 joint-limit CBF를 합친 `(lower, upper)` |
| `collision_velocity_barriers(constraints, dt, *, safe_distance, gain)` | `VelocityBarrier` tuple |
| `clip_joint_positions(position, limited, ranges)` | 제한 관절만 model range로 자른 복사본 |

## Kinematic tree와 FK { #tree }

모듈은 `kinematics.tree`다.

| 자료형 | 주요 값 |
|---|---|
| `SiteKinematics` | world `position`, `quaternion`, 6×N `jacobian` |
| `KinematicJoint` | joint id/name/type, qpos·DOF 주소, 축, 제한과 범위 |
| `KinematicBody` | body id/name, parent, 고정 변환, joint id |
| `KinematicSite` | site id/name, body id, 고정 변환 |

### `KinematicTree(model)`

컴파일된 `MjModel`의 body-joint-site 구조를 복사한다. 생성 뒤 FK는 `MjData`나
`mj_forward()` 없이 입력 `qpos`로 계산한다.

| 메서드 | 입력 | 반환 |
|---|---|---|
| `forward_site(qpos, site_id, joint_ids)` | 전체 qpos, site id, Jacobian 열의 joint 순서 | `SiteKinematics` |
| `point_jacobian(qpos, body_id, point_world, joint_ids, frame_cache=None)` | body 위 world 점 | shape `(3,N)` 선속도 Jacobian |

## Differential IK solver { #solver }

모듈은 `kinematics.solver`다.

### `IKMethod`

`PSEUDOINVERSE`, `DLS`, `QP`를 제공한다. `IKMethod.coerce(value)`는 `pinv`,
`damped_least_squares`, `quadratic_program` 등의 별칭을 열거형으로 변환한다.

### `DifferentialIKSolver(...)`

```python
solver = DifferentialIKSolver(method="qp")
qdot = solver.solve(matrix, vector, lower, upper)
```

| 메서드 | 동작 |
|---|---|
| `set_method(method)` | 다음 계산에 사용할 해법 변경 |
| `solve(matrix, vector, lower, upper)` | weighted least-squares와 box bound를 만족하는 `qdot` 반환 |
| `enforce_constraints(reference, lower, upper, barrier_matrix=None, barrier_lower=None, barrier_weight=1.0, *, variable_scale=None, barrier_scale=1.0)` | 기준 속도에 box와 soft barrier를 적용한 보정 속도 반환 |

Pseudoinverse와 DLS도 포화된 축을 고정하고 남은 축으로 residual을 다시 계산한다.
`enforce_constraints()`는 명목 해법과 별도로 사용하는 safety projection이다.

## QP 수치 함수 { #optimization }

모듈은 `kinematics.optimization`이다.

| 함수 | 문제와 반환 |
|---|---|
| `least_squares_to_qp(matrix, vector)` | `||Ax-b||²` → `(hessian, linear)` |
| `bounded_quadratic_program(hessian, linear, lower, upper)` | box-constrained convex QP 해 |
| `bounded_quadratic_program_with_barriers(..., barrier_matrix, barrier_lower, slack_weight)` | `Gx >= h` squared-hinge 비용을 더한 box-QP 해 |

soft barrier 함수는 별도 slack 변수를 반환하지 않는다. 현재 위반 행의 quadratic
비용을 추가하고 활성 위반 집합이 안정될 때까지 다시 계산한다.

## 충돌 거리 { #collision }

모듈은 `kinematics.collision`이다.

| 자료형 | 필드 |
|---|---|
| `CollisionPair` | `name`, `geom_a`, `geom_b`, `mode` |
| `CollisionConstraint` | `name`, signed `distance`, `gradient`, `point_a`, `point_b` |

| 함수 | 동작·반환 |
|---|---|
| `default_collision_pairs(model)` | 기본 self/table/can 감시 pair tuple |
| `collision_distance_gradient(model, data, pair, tree, joint_ids, max_distance, frame_cache=None)` | 범위 안의 `CollisionConstraint`, 아니면 `None` |

기본 pair는 wheel-floor와 finger-object처럼 의도한 접촉을 제외한다. 거리 gradient의
유도는 [Collision distance와 gradient](../guide/collision-kinematics.md)에 있다.

## 이전 이름 호환 { #legacy }

`kinematics.legacy.KinematicsSolver`는 단일 site의 `forward()`와 같은 이름인
`forward_kinematics()`만 제공한다. `from_mjcf()`로 모델을 읽을 수 있다.
`InverseKinematics`는 같은 FK adapter 이름이며 반복형 pose solve는 제공하지 않는다.
새 코드에서는 `KinematicTree`를 사용한다.

## MuJoCo 공통 함수 { #mujoco-utils }

`ffw_sh5_grasp.mujoco_utils.find_actuator_for_joint(model, joint_id)`는 연결된 actuator id를
반환하고, 없으면 `None`을 반환한다. `None`을 확인하지 않고 `data.ctrl` index로 쓰면
NumPy broadcasting이 발생할 수 있다.

수식과 구현 흐름은 [기구학 학습 안내](../guide/kinematics.md)를 참고한다.
