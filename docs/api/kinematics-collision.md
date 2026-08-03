# `kinematics.collision`

감시 geometry의 signed distance와 WBIK 관절 열에 대한 거리 gradient를 계산한다.

## `CollisionPair`

두 geom과 거리 계산 mode를 정의한다. 일반 geom pair, table-top 및 bounding-sphere
근사를 표현할 수 있다.

## `CollisionConstraint`

| 필드 | 의미 |
|---|---|
| `name` | 감시 pair 이름 |
| `distance` | signed distance, m |
| `gradient` | 전체 WBIK 열의 거리 gradient |
| `point_a`, `point_b` | 두 형상의 world 최근접점 |

## `default_collision_pairs(model)`

- **기능:** self/table/can 중 CBF가 감시할 pair를 모델 이름으로 구성한다.
- **반환:** `CollisionPair` tuple.
- **제외:** 손가락–물체와 wheel–floor 같은 의도된 접촉.

## `collision_distance_gradient(...)`

```python
collision_distance_gradient(
    model, data, pair, tree, joint_ids,
    max_distance, frame_cache=None,
)
```

- **기능:** 현재 거리와 거리를 늘리는 관절 방향을 동시에 계산한다.
- **입력:** model/data, pair, 공유 tree, WBIK joint 열 순서, 검색 거리.
- **반환:** 범위 안이면 `CollisionConstraint`, 멀거나 유효하지 않으면 `None`.
- **사용:** collision CBF와 동일 constraint의 화면 표시.

최근접점 Jacobian에서 `ḋ = ∇d q̇`까지의 유도는
[Collision distance와 gradient](../guide/collision-kinematics.md)를 참고한다.
