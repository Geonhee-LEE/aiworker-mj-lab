# `kinematics.tree`

컴파일된 MuJoCo model의 body–joint–site 고정 구조를 복사하고, 임의 `qpos`의 FK와
Jacobian을 계산한다.

## 결과와 구조 자료형

### `SiteKinematics`

| 필드 | 의미 |
|---|---|
| `position` | world 위치, shape `(3,)`, m |
| `quaternion` | world 자세, shape `(4,)`, `wxyz` |
| `jacobian` | 선택 관절의 geometric Jacobian, shape `(6,N)` |

`KinematicJoint`, `KinematicBody`, `KinematicSite`는 각각 관절 축·범위, body
고정변환·부모, site 고정변환을 보관하는 불변 구조 레코드다.

## `KinematicTree(model)`

- **기능:** root에서 목표 body/site까지 따라갈 기구학 지도를 한 번 생성한다.
- **입력:** `mujoco.MjModel`.
- **보관:** body/joint/site 구조, 이름 lookup, 기본 `qpos0`.
- **부작용:** 없음.

### `KinematicTree.forward_site(qpos, site_id, joint_ids)`

- **기능:** root부터 site까지 변환을 누적하고 pose와 Jacobian을 함께 계산한다.
- **입력:** 모델 전체 `qpos`, site id, Jacobian 열 순서의 joint id 목록.
- **반환:** world-frame `SiteKinematics`.
- **주의:** 입력 배열과 live `MjData`를 수정하지 않는다.

### `KinematicTree.point_jacobian(qpos, body_id, point_world, joint_ids, frame_cache=None)`

- **기능:** body 원점이 아닌 충돌 최근접점의 선속도 Jacobian을 계산한다.
- **반환:** shape `(3,N)` world Jacobian.
- **사용:** geometry distance gradient 계산.

Tree 생성 이유는 [Kinematic Tree](../guide/kinematic-tree.md), Jacobian 열의 유도는
[FK와 Geometric Jacobian](../guide/forward-kinematics.md)을 참고한다.
