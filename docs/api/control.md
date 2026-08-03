# 제어 API

기구학 결과를 generalized velocity, 관절 목표, 바퀴와 손가락 actuator 명령으로
바꾸는 계층이다.

| 모듈 | 책임 | 상세 문서 |
|---|---|---|
| `control.whole_body` | 18-DOF 전신/arm-only 명령 조립 | [전신 IK API](control-whole-body.md) |
| `control.optimization` | 명시적 box-QP와 collision soft barrier | [최적화 API](control-optimization.md) |
| `control.bimanual` | 양손 상대 pose 캡처와 task | [양손 API](control-bimanual.md) |
| `control.arm` | Bias 보상 PD 팔 토크 | [팔 제어 API](control-arm.md) |
| `control.base` | 키보드 속도와 스워브 기구학/FSM | [베이스 API](control-base.md) |
| `control.grasp` | 손가락 synergy와 접촉 판정 | [파지 API](control-grasp.md) |

`WholeBodyIK.solve()`는 명령을 반환하고 actuator를 직접 쓰지 않는다. `arm`과
`grasp`의 적용 함수가 `data.ctrl`을 기록한다.
