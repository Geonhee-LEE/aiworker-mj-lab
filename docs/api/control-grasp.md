# `control.grasp`

두 synergy 값을 손가락 관절 목표로 펼치고 실제 접촉력으로 파지를 판정한다.

## `apply_grasp(model, data, grasp, thumb, side="r")`

- **입력:** curl `grasp`, 엄지 curl `thumb`은 0~1, `side`는 `"r"`/`"l"`.
- **변경:** 해당 손가락 position actuator의 `data.ctrl`.
- **특징:** 입력을 `[0,1]`로 제한하고 model별 계수는 캐싱한다.

## `get_finger_can_contacts(model, data, side="r")`

- **기능:** 캔과 닿은 손가락 그룹별 법선 힘을 합산한다.
- **반환:** `{finger_group: summed_normal_force}` mapping, N.

## `is_grasped(...)`

```python
is_grasped(
    model, data, min_fingers=2, min_total_force=0.05,
    require_thumb=True, side="r",
)
```

- **기능:** 목표값이 아니라 실제 접촉 그룹 수와 힘으로 성공을 판정한다.
- **반환:** 모든 조건을 만족하면 `True`.

Synergy affine 식과 좌우 mirror 처리는
[손 파지와 접촉 판정](../guide/grasp.md)을 참고한다.
