# `kinematics.rotations`

모델이나 solver 상태를 갖지 않는 공용 회전·벡터 함수다.

| 함수 | 기능 | 입력 | 반환 |
|---|---|---|---|
| `normalize_quaternion(quaternion)` | 길이와 부호를 안정적인 대표값으로 정리 | 4-vector | 단위 `wxyz`; 잘못된 norm은 identity |
| `multiply_quaternions(*quaternions)` | 적힌 순서대로 여러 회전 합성 | 하나 이상의 `wxyz` | 합성 `wxyz` |
| `inverse_quaternion(quaternion)` | 반대 방향 회전 생성 | 단위 `wxyz` | 켤레 `wxyz` |
| `rpy_deg_to_quat(rpy_deg)` | Roll/Pitch/Yaw를 solver 자세로 변환 | degree 3-vector | `wxyz` |
| `quat_to_rpy_deg(quaternion)` | Solver 자세를 UI 각도로 변환 | `wxyz` | degree RPY |
| `shortest_orientation_error(target, current)` | 현재→목표 최단 자세 오차 | 두 `wxyz` | world axis-angle, rad |
| `rotation_from_quaternion(quaternion)` | Quaternion을 점·축 계산 행렬로 변환 | `wxyz` | `(3,3)` 회전행렬 |
| `quaternion_from_rotation(rotation)` | 회전행렬을 정규화 Quaternion으로 변환 | `(3,3)` 행렬 | 단위 `wxyz` |
| `axis_rotation(axis, angle)` | Rodrigues 축 회전 계산 | 3D 축, rad | `(3,3)` 행렬 |
| `clip_norm(vector, limit)` | 방향을 유지하며 norm 제한 | vector, limit | 제한된 복사본 |
| `wrap_angle(angle)` | 각도를 한 바퀴 범위로 접기 | rad | `[-π, π)` |
| `skew(vector)` | 외적을 행렬 곱으로 표현 | 3-vector | `(3,3)` skew 행렬 |

자세 오차의 곱셈 순서와 이중 표현 처리는
[Quaternion과 자세 오차](../guide/quaternion-math.md)를 참고한다.
