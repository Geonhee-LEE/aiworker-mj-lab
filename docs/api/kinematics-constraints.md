# `kinematics.constraints`

Differential IK의 hard 제약을 만드는 순수 NumPy API다. 로봇 상태와 solver를 소유하지
않으며 모든 출력 속도 단위는 입력 generalized velocity와 같다.

| API | 의미 |
|---|---|
| `joint_velocity_bounds(...)` | 물리 속도 상한과 joint-limit CBF를 box bound로 결합 |
| `collision_velocity_barriers(...)` | distance gradient를 `gradient @ qdot >= lower`로 변환 |
| `clip_joint_positions(...)` | 제한된 관절의 적분 결과를 최종 model range로 clip |
| `VelocityBarrier` | 이름, 거리, gradient, lower bound 진단값 |

joint-limit과 collision을 같은 파일에 둔 이유는 둘 다 **현재 상태를 generalized
velocity 제약으로 바꾸는 계층**이기 때문이다. 최근접점과 gradient 자체는
[`kinematics.collision`](kinematics-collision.md)이 계산한다.
