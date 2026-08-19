# 애플리케이션과 목표 좌표

`application/teleop.py`는 model/data와 controller를 만들고 프레임 반복 실행을 담당한다.
`application/control_loop.py`는 수동 주행과 IK 명령을 중재하며, `application/targets.py`는
UI 값, world pose와 marker pose 사이의 변환을 담당한다. `application/state.py`는 모델
주소와 한 frame의 입력·출력 스냅샷을 정의한다.

```bash
python3 src/teleop_app.py
python3 src/teleop_app.py --config config/local.yaml
```

## 모듈별 역할

| 파일 | 역할 | 하지 않는 일 |
|---|---|---|
| `application/teleop.py` | 초기화, 입력, 모드 전환, frame·physics 순서 조율 | FK·IK 수식 중복 구현 |
| `application/control_loop.py` | base feedback, 수동/WBIK 중재, solver 결과 반영 | UI·렌더링 |
| `application/targets.py` | target frame 변환, 양손 capture, marker 동기화 | IK solve, actuator 기록 |
| `application/state.py` | 모델 주소와 task/control/observation 자료구조 | solver 계산 |

루트의 `src/teleop_app.py`는 설정 경로를 적용한 뒤
`ffw_sh5_grasp.application.teleop.main()`을 호출하는 실행기다.

## 실행 흐름

```mermaid
flowchart TD
    MAIN["src/teleop_app.py<br>main()"] --> APP["application/teleop.py<br>TeleopApp.__init__()"]
    APP --> SETUP["_setup_sim()<br>model → controller → ID → target"]
    SETUP --> LOOPSTATE["_setup_loop_state()"]
    LOOPSTATE --> RUN["TeleopApp.run()"]
    RUN --> BEGIN["visualization/render.py<br>begin_frame()"]
    BEGIN --> INPUT["_handle_edge_keys()<br>_read_drive_and_lift_keys()"]
    INPUT --> UI["visualization/ui.py<br>draw_panel()"]
    UI --> STEP["TeleopApp._step_physics()"]
    STEP --> MANUAL["application/control_loop.py<br>update_manual_drive()"]
    MANUAL --> TARGET["application/targets.py<br>target 변환"]
    TARGET --> TASK["application/state.py<br>TaskCommand.create()"]
    TASK --> APPLY["application/control_loop.py<br>apply_whole_body_solution()"]
    APPLY --> IK["control/whole_body.py<br>WholeBodyIK.solve()"]
    IK --> SELECT["application/control_loop.py<br>select_base_command()"]
    SELECT --> BASE["control/base.py<br>SwerveDrive.update_twist()"]
    BASE --> COMMAND["build_control_command()<br>ControlCommand"]
    COMMAND --> ACT["_step_actuators()<br>data.ctrl · mj_step()"]
    ACT --> OBS["TeleopApp.observe()<br>RobotObservation"]
    OBS --> RENDER["visualization/render.py<br>render_scene() · end_frame()"]
    RENDER --> RUN
```

### `_step_physics()` 순서

1. `update_manual_drive()`가 base feedback을 읽고 수동 주행 상태를 갱신한다.
2. 수동 주행 중이면 측정된 base SE(2) 이동만큼 target 기준을 운반한다.
3. grasp target과 손 target 변화율을 제한한다.
4. 양손 world pose와 lift·손 값을 `TaskCommand`로 묶어 solver에 전달한다.
5. `select_base_command()`가 manual, 제동용 zero, WBIK 중 하나를 선택한다.
6. 결과를 `ControlCommand`로 묶어 각 physics substep에 적용한다.
7. `mj_step()` 뒤 상태를 `RobotObservation`으로 복사한다.

base 명령 중재와 wheel 제어는 [모바일 스워브 제어](base_teleop.md#base-function-flow)에
정리되어 있다.

## 목표 좌표계 { #target-frames }

| 저장값 | Whole-body ON | Whole-body OFF |
|---|---|---|
| 손 `pos/rpy` | startup 또는 수동 주행으로 운반된 anchor 기준 | 현재 live base 기준 |
| virtual object | anchor의 절대 local pose | live base의 절대 local pose |
| solver 입력 | world pose | world pose |

ON에서는 target이 world에 고정되어야 base가 움직여 task error를 줄일 수 있다. 손 위치
목표는 시작 손 위치와 anchor 방향을 사용한다.

\[
p_{target}^{w}=p_{home}^{w}+R_z(\theta_{anchor})\,\Delta p_{ui}
\]

virtual object 위치는 손 offset과 달리 anchor-local 절대 위치이므로 anchor 원점도
더한다.

\[
p_{object}^{w}=
\begin{bmatrix}x_{anchor}\\y_{anchor}\\0\end{bmatrix}
+R_z(\theta_{anchor})p_{object}^{anchor}
\]

<figure markdown>
  ![World, startup anchor, 이동한 live base와 world-fixed 손 target의 관계](../assets/target-frames.svg)
  <figcaption>Whole-body ON의 손 target은 live base가 아니라 anchor를 기준으로 world에 고정된다.</figcaption>
</figure>

자세도 같은 구분을 사용한다.

\[
q_{world}^{ON}=q_{home\_world}\otimes q_{rpy},\qquad
q_{world}^{OFF}=q_{base\_world}\otimes q_{home\_base}\otimes q_{rpy}
\]

`set_whole_body_enabled()`는 전환 전에 손과 virtual object의 world pose를 저장하고,
전환 후 새 좌표계의 값으로 역변환한다. 따라서 UI 숫자는 달라질 수 있지만 marker의
world pose는 유지된다. FK→IK도 현재 site pose를 새 target으로 사용해 전환 점프를
막는다.

## 양손 capture

`capture_grasp()`는 virtual object에서 본 두 손의 상대 transform을 저장한다.

\[
p_{offset}=R_{obj}^{T}(p_{hand}-p_{obj}),\qquad
R_{offset}=R_{obj}^{T}R_{hand}
\]

이후 `apply_virtual_object_target()`은 반대 변환으로 두 손 target을 갱신한다.

\[
p_{hand}=p_{obj}+R_{obj}p_{offset},\qquad
R_{hand}=R_{obj}R_{offset}
\]

`release_grasp()`는 저장된 상대 transform과 solver의 rigid-grasp 기준을 함께 해제한다.

## Target 변환 흐름

```mermaid
flowchart LR
    NUM["Task Space 숫자"] --> APPLYNUM["visualization/task_space.py<br>apply_target()"]
    APPLYNUM --> STATE["app.targets"]
    EDIT["slider · jog"] --> STATE
    GIZMO["visualization/render.py<br>draw_transform_gizmo()"] --> SET["application/targets.py<br>set_gizmo_target_world_pose()"]
    SET --> STATE
    STATE --> POSE["target_world_pose()"]
    POSE --> SOLVE["control/whole_body.py<br>WholeBodyIK.solve()"]
    STATE --> MOCAP["sync_ik_mocaps_from_targets()"]
    MOCAP --> VIEW["MuJoCo target marker"]

    CAP["capture_grasp()"] --> OFFSET["object 기준 손 offset"]
    OFFSET --> APPLY["apply_virtual_object_target()"]
    APPLY --> STATE
```

수동 주행은 `carry_world_targets_with_base()`로 anchor와 손 home reference를 측정된 base
이동만큼 옮긴다. 이 과정은 UI offset을 직접 수정하지 않는다.

## 주요 함수

| 함수 | 역할 |
|---|---|
| `TeleopApp.run()` | 입력 → UI → 물리 → 렌더링 프레임 반복 |
| `TeleopApp.set_arm_mode()` | IK/FK 전환과 현재 상태 동기화 |
| `TeleopApp.set_whole_body_enabled()` | world pose를 보존하며 ON/OFF 전환 |
| `TeleopApp._step_physics()` | target, solver, 명령 중재와 physics 순서 조율 |
| `TeleopApp.observe()` | 현재 상태를 독립된 `RobotObservation`으로 복사 |
| `control_loop.update_manual_drive()` | base feedback과 수동 우선권·target 운반 갱신 |
| `control_loop.apply_whole_body_solution()` | solver 호출 결과를 팔·리프트·진단 상태에 반영 |
| `control_loop.select_base_command()` | manual/제동/WBIK 명령 선택과 wheel 명령 생성 |
| `control_loop.build_control_command()` | 물리 적용용 명령 스냅샷 생성 |
| `target_world_pose()` | 손 target을 최종 world pose로 변환 |
| `world_to_target_pos()` | world 위치를 현재 target frame으로 역변환 |
| `capture_grasp()` / `apply_virtual_object_target()` | virtual object 기준 양손 target 저장·복원 |
| `sync_ik_mocaps_from_targets()` | target pose를 표시용 mocap에 복사 |

`_step_actuators()`만 robot actuator의 `data.ctrl`을 기록한다. `reset_can()`과 legacy
box 비활성화는 자유물체 초기화를 위해 해당 object의 state/model 속성을 수정한다.

## 검증

```bash
python3 tests/test_phase_6.py
```

Phase 6는 초기 target 일치, world↔target 변환, Whole-body 전환 pose 보존, 양손 capture,
marker 동기화, Task Space 입력과 상태·명령 스냅샷을 검사한다.

[← 시스템 구조](../overview.md) · [시각화 →](teleop_ui.md)
