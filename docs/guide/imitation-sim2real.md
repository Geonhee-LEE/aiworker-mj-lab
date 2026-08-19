# 모방학습 데이터와 실기 전환

!!! warning "현재 구현되지 않음"
    episode recorder, dataset loader, policy runner와 실물 driver는 없다.
    `tests/record_demo.py`는 GIF 생성용 테스트 도구다. 아래 인터페이스는 확장 시의
    제안이며 현재 API가 아니다.

## 권장 policy 출력

첫 버전은 actuator나 torque 대신 손 pose 변화량과 finger synergy를 출력하는 편이
현재 안전 계층을 재사용하기 쉽다.

\[
a_t=[\Delta p_r,\Delta\theta_r,\Delta p_l,\Delta\theta_l,
\Delta g_r,\Delta t_r,\Delta g_l,\Delta t_l]
\]

| 값 | 단위·frame |
|---|---|
| \(\Delta p\) | task frame 위치 변화량, m |
| \(\Delta\theta\) | 같은 frame의 rotation vector, rad |
| \(\Delta g,\Delta t\) | `grasp`, `thumb` 변화량 |

이 action은 `application.targets`를 거쳐 기존 target smoothing, WBIK, collision CBF,
swerve와 저수준 팔 제어를 사용한다.

## 기록할 데이터

`TeleopApp._step_physics()`가 한 control frame의 입력과 최종 명령을 모두 보는 조립
지점이다. 계산 모듈에서 직접 파일을 쓰지 말고 recorder에 snapshot을 넘긴다.

```text
observation_t
  + raw human target/event
  + normalized policy action
  + WholeBodyCommand
  + wheel/finger/actuator command
  -> physics substeps
  -> observation_t+1
```

### Episode metadata

- `schema_version`, episode ID, git commit, model hash
- seed와 reset 값
- control Hz와 physics dt
- joint·actuator·camera 이름 순서
- controller config
- task, success, failure reason

### Step record

- sequence, monotonic/simulation timestamp, dt, dropped-frame 표시
- qpos, qvel, base pose/twist, wheel feedback
- 양손·물체 pose와 상대 pose
- 접촉력, 최소 충돌 거리, 활성 pair, CBF 위반량
- Whole-body/IK/FK/Bimanual/manual mode
- raw target, 학습 action, solver 출력, 최종 actuator command
- 선택적 RGB/depth frame과 sensor timestamp

`observation_t`를 읽고 action을 적용한 결과는 `observation_t+1`에 저장한다. 기본
표본 주기는 physics 1 kHz가 아니라 control 25 Hz다. 배열 순서는 metadata의 이름
목록으로 고정하고 episode 단위 완료 표시와 schema version을 둔다.

## 데이터 검증

- timestamp와 sequence가 단조 증가하는가
- 모든 수치가 finite이고 quaternion norm이 1인가
- joint/actuator 배열 길이와 이름 순서가 고정됐는가
- action과 다음 observation이 한 step 어긋나지 않았는가
- target·velocity·command가 코드 limit 안에 있는가
- 수동 개입과 collision CBF 개입이 표시됐는가
- train/validation/test를 frame이 아니라 episode 단위로 나눴는가

리플레이는 target action을 현재 controller에 다시 넣는 closed-loop 방식을 기본으로
한다. actuator open-loop replay는 dynamics 변경 감지용이며 장기 궤적 일치를 기대하지
않는다. 초기 상태 외에 robot qpos를 기록값으로 계속 덮어쓰지 않는다.

## 실기 전환 경계

| 현재 모듈 | 실기에서 필요한 변경 |
|---|---|
| `application/targets.py`, `rotations.py` | frame·단위 계약이 같으면 재사용 가능 |
| `KinematicTree`, legacy `KinematicsSolver` | 실제 축·zero·tool pose와 일치할 때 FK/Jacobian만 재사용 |
| `WholeBodyIK.solve(MjData, ...)` | shadow model 또는 observation adapter 필요 |
| `kinematics/collision.py` | calibrated scene나 실기 collision source 필요 |
| `SwerveDrive` | wheel 위치·반지름·부호·gear·feedback 보정 필요 |
| `ArmTorqueController.apply()` | MuJoCo bias/ctrl 전용이므로 실물에 직접 사용 금지 |
| `grasp.apply_grasp()` | hand actuator와 current/force limit adapter 필요 |

제안하는 hardware 경계는 측정, 명령, 정지, fault reset만 노출한다.

```python
class RobotHardware:
    def read_observation(self): ...
    def write_command(self, command): ...
    def stop(self): ...
    def reset_faults(self): ...
```

vendor SDK, CAN/EtherCAT 또는 ROS2 연결은 이 경계 아래에 둔다.

## 실기 전 필수 확인

- world/base/tool/camera frame, 길이 단위와 quaternion 순서
- encoder zero·joint sign·gear ratio·관절 한계
- tool center와 collision geometry 위치
- sensor timestamp, 통신 지연·jitter·packet loss
- wheel slip, 마찰, backlash, torque/current limit
- stale-command 거부, watchdog, E-stop

정책 출력은 항상 다음 안전 경로를 통과해야 한다.

```text
policy action
→ workspace·joint·속도·가속도 제한
→ collision 접근 속도 제한
→ command rate limit
→ hardware command
→ watchdog·E-stop
```

WBIK의 CBF는 미모델링 장애물과 사람을 감지하지 않으므로 단독 안전 장치가 아니다.

## 배포 순서

1. dataset schema와 deterministic target replay 검증
2. 보지 않은 reset과 dynamics randomization에서 simulation 평가
3. 명령을 보내지 않는 실물 shadow mode로 frame·zero·지연 확인
4. 무부하·저속 단축 시험으로 sign, limit, watchdog, E-stop 확인
5. base/팔 분리 시험 후 저속 teleop 시험
6. operator dead-man switch를 유지한 짧은 policy trial

각 단계에서 frame, model, 저수준 추종, perception, policy 오류를 구분해 기록한다.

[API 레퍼런스](../api/index.md)는 현재 구현된 함수만 다룬다. 기존 simulation 최소
회귀는 [테스트와 검증](../testing.md)을 따른다.
