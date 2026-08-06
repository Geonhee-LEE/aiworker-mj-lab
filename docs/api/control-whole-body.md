# `control.whole_body`

Base x/y/yaw, lift와 양팔 14축의 task·제약을 하나의 differential IK 문제로 조립한다.
실제 pseudoinverse·DLS·QP 계산은 `kinematics.solver`에 위임한다.

## `WholeBodyIK(model, site_names, arm_joint_names, **weights)`

- **입력:** `MjModel`, 손별 site·관절 이름, 선택적 task/CBF 가중치와 gain,
  `base_participation_scale`.
- **상태:** 공유 `KinematicTree`, 속도 제한, base·손 reference, rigid-grasp reference.
- **설정:** `config/default.yaml`의 `whole_body_ik`.
- **베이스 참여:** `base_participation_scale=0.05`는 base 목표와 속도 상한을 기본값의
  5%로 낮추고, `0.0`은 base 3축만 hard pin하면서 lift·팔은 유지한다.
- **유효 범위:** `base_participation_scale`은 `0.0` 이상 `1.0` 이하다.
- **해법 선택:** `solver_method="pseudoinverse" | "dls" | "qp"`. 생략하면 YAML의
  `whole_body_ik.solver.method`를 사용한다.

## `WholeBodyIK.solve(...)`

```python
solve(
    data, target_poses, dt,
    *, active_sides=("r", "l"), arm_nominal=None,
    lift_nominal=None, rigid_grasp=False,
    whole_body_enabled=True,
)
```

- **기능:** 현재 상태와 world 손 목표에서 다음 frame 명령을 계산한다.
- **반환:** `WholeBodyCommand`.
- **OFF:** `whole_body_enabled=False`이면 base·lift 속도를 0으로 hard pin한다.
- **부작용:** actuator와 live `data.qpos`를 쓰지 않는다.
- **공통 규칙:** 손 pose 오차와 목표 twist는
  [`kinematics.tasks`](kinematics-tasks.md)의 공통 함수로 만든다.
- **QP 비용:** 모든 task·정규화·slack 값은 대응 속도 상한으로 정규화한 무차원
  strength다. 실제 비용은 `strength * (residual / speed_scale)²`이다.
- **QP 베이스 bound:** 생성자/YAML의 베이스 참여 설정은 ON 상태의 base 3축 속도
  bound에 적용된다. `whole_body_enabled=False`는 이 설정과 무관하게 base와 lift를
  모두 고정한다.
- **수치 경로:** 이 클래스는 task 행렬·벡터와 bound를 조립한 뒤
  `DifferentialIKSolver.solve()`를 호출한다.

### 실행 중 solver 설정

| 메서드 | 역할 |
|---|---|
| `set_solver_method(method)` | pseudoinverse, DLS, QP 선택 |
| `set_dls_damping(value)` | DLS 감쇠 갱신 |
| `qp_weights()` | UI 편집 가능한 QP 가중치 사전 반환 |
| `set_qp_weight(name, value)` | task·정규화·posture·collision slack 가중치 갱신 |

### QP strength의 의미

| 이름 | 무차원화하는 잔차 | 값이 커질 때 |
|---|---|---|
| `position` | 손 선속도 오차 / 최대 task 선속도 | 위치 추종 우선 |
| `orientation` | 손 각속도 오차 / 최대 task 각속도 | 방향 추종 우선 |
| `rigid_grasp_position` | 양손 상대 선속도 오차 / 최대 task 선속도 | 상대 위치를 더 강하게 보존 |
| `rigid_grasp_orientation` | 양손 상대 각속도 오차 / 최대 task 각속도 | 상대 방향을 더 강하게 보존 |
| `damping_*` | 해당 자유도 속도 / 해당 속도 상한 | 해당 자유도가 덜 움직임 |
| `posture_*` | nominal 복귀 속도 오차 / 해당 속도 상한 | 기준 자세로 더 강하게 복귀 |
| `collision_slack` | collision CBF 위반 속도 / 최대 task 선속도 | 안전 위반을 더 비싸게 취급 |

이 항목들은 collision의 soft slack을 포함한 소프트 목적함수다. 속도 bound,
joint-limit CBF가 만든 bound, Whole-body OFF/FK 모드의 고정 자유도는 별도의 하드
제약이라 slider를 낮춰도 위반하지 않는다.

## `WholeBodyCommand`

| 필드 | 의미 |
|---|---|
| `base_twist` | 스워브 제어기에 전달할 차체 좌표 속도 |
| `arm_positions` | 손별 다음 목표 관절각 |
| `lift_position` | 다음 리프트 목표 위치 |
| `position_errors`, `orientation_errors` | 손별 pose 오차 norm |
| `generalized_velocity` | `[base(3), lift, right(7), left(7)]` 해 |
| `minimum_collision_distance` | 활성 pair의 최소 signed distance |
| `active_collision_pairs` | 이번 frame의 CBF 입력 |
| `collision_constraint_violation` | 최대 soft-barrier 위반량 |

## 상태와 진단 메서드

### `WholeBodyIK.rebase(data, target_poses=None)`

현재 상태를 새 출발점으로 저장하고 이전 base 속도·solve 시간을 초기화한다.

### `WholeBodyIK.set_rigid_grasp(data, active)`

현재 양손 상대 pose를 저장하거나 기존 rigid-grasp reference를 해제한다.

### `WholeBodyIK.site_state(data, side, current_q=None)`

WBIK와 같은 관절 열 순서의 한 손 `SiteKinematics`를 반환한다.

### `WholeBodyIK.collision_distances(data, max_distance=None)`

CBF와 같은 pair 정의를 사용해 가까운 `CollisionConstraint` tuple을 반환한다.

수식과 task 조립은 [전신 IK와 충돌 회피](../guide/whole_body_ik.md)를 참고한다.
