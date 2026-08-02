# 시스템 이해와 개발 가이드

이 페이지는 시스템의 동작 원리를 이해하는 경로와 코드를 안전하게 수정하는 경로를
하나로 모은 안내서다. 앱을 아직 실행하지 않았다면 [빠른 시작](../getting-started.md)을
먼저 따라 해보는 편이 이해가 빠르다.

## 읽기 경로 선택

시스템 개념, 코드 중심 설명과 ROS2 관점 설명을 별도 가이드로 나누지 않는다. 아래에서
익숙한 출발점을 고른 뒤 모두 같은 아키텍처와 모듈별 문서로 이어진다.

| 목적 | 권장 순서 |
|---|---|
| 시스템 동작 이해하기 | [동작 원리](../concepts.md) → [아키텍처와 데이터 흐름](../overview.md) |
| 처음 코드를 읽기 | [동작 원리](../concepts.md) → [아키텍처](../overview.md) → 이 페이지의 코드 계층 |
| 수정할 파일 찾기 | [수정 목적별 경로](#change-paths) → 해당 모듈 가이드 → [테스트](../testing.md) |
| ROS2 경험으로 이해하기 | [ROS2 관점으로 읽기](#ros2-reading)에서 목적별 Part 선택 |
| 제어 수학 이해하기 | [핵심 알고리즘 학습 순서](#algorithm-learning-order)를 1번부터 따라가기 |
| 좌표계·3D 조작 이해하기 | [목표와 좌표 변환](teleop_targets.md) → [ROS2 Part 9·10](#coordinate-reading) |
| 모방학습·실기 전환 준비 | [모방학습 데이터와 실기 전환](imitation-sim2real.md) |

!!! tip "사용법을 찾는 중이라면"
    키와 버튼은 [화면과 조작](../run.md), 모드 조합은
    [모드 선택](../control-modes.md), 증상 진단은
    [문제 해결](../troubleshooting.md)이 더 빠르다.

## 전체 완독 순서

처음부터 끝까지 한 번 읽을 때는 아래 순서를 권장한다. 핵심 알고리즘 안에서는
바로 다음 절의 1A~7 순서를 그대로 따른다.

1. [빠른 시작](../getting-started.md)으로 실제 화면과 기본 동작을 확인한다.
2. [동작 원리](../concepts.md) → [아키텍처와 데이터 흐름](../overview.md) →
   [MuJoCo 기본 용어](00-basics.md)로 상태와 모듈 경계를 잡는다.
3. [목표와 좌표 변환](teleop_targets.md)에서 UI 입력이 world hand pose가 되는
   과정을 읽는다.
4. [핵심 알고리즘 학습 순서](#algorithm-learning-order)의 1A~7을 읽어
   world pose가 관절·바퀴·손가락 명령으로 바뀌는 수학을 따라간다.
5. [앱 조립과 물리 루프](teleop_app.md) → [UI 패널](teleop_ui.md) →
   [렌더링과 Gizmo](teleop_render.md)로 실제 한 frame의 호출 관계를 확인한다.
6. [API 치트시트](cheatsheet.md)에서 작업별 공개 함수를 고르고,
   [개발 체크리스트](pitfalls.md)와 [테스트와 검증](../testing.md)으로 마무리한다.

이후 데이터를 수집하거나 실물 로봇으로 확장할 때만
[모방학습 데이터와 실기 전환](imitation-sim2real.md)을 읽는다. 이 페이지는 현재
구현과 아직 추가해야 할 recorder·hardware adapter의 경계를 구분한다.

이 경로를 마치면 **입력 → 좌표 변환 → 기구학/IK → actuator 명령 → 물리 → 렌더링**을
파일과 함수 단위로 추적할 수 있다. ROS2 비교가 필요한 독자만 관련 Part 링크를
보조 설명으로 읽으면 되며, 같은 내용을 처음부터 다시 읽을 필요는 없다.

## 핵심 알고리즘 학습 순서 { #algorithm-learning-order }

수식의 입력을 만드는 계층부터 해를 구하는 계층, 해를 물리 actuator에 적용하는
계층 순으로 읽는다.

```mermaid
flowchart LR
    T["1A. Tree<br>구조와 경로"] --> F["1B. FK/Jacobian<br>pose와 변화율"]
    F --> Q["1C. Quaternion<br>자세 표현과 오차"]
    Q --> C["1D. Collision<br>distance gradient"]
    C --> M["2. DLS 수학<br>목적함수 · 유도"]
    M --> I["3. 단일 팔 IK<br>반복 solver"]
    I --> W["4. 전신 IK<br>bounded solve · CBF"]
    W --> A["5. 팔 토크<br>q_des → torque"]
    W --> B["6. 스워브<br>BodyTwist → wheel"]
    A --> G["7. 파지<br>synergy · contact"]
    B --> G
```

| 순서 | 먼저 답할 질문 | 핵심 수식 | 구현 문서 |
|---:|---|---|---|
| 1A | 모델 구조를 왜 별도 tree로 만들고 FK에 무엇을 넘기는가? | root→target body path | [Kinematic Tree](kinematic-tree.md) |
| 1B | tree의 joint 변환이 pose와 Jacobian으로 어떻게 이어지는가? | \(x=f(q),\ \dot x=J(q)\dot q\) | [FK와 Jacobian](forward-kinematics.md) |
| 1C | 회전 표현의 부호와 world-frame 자세 오차를 어떻게 정하는가? | \(q_e=q_t\otimes q_c^{-1}\) | [Quaternion과 자세 오차](quaternion-math.md) |
| 1D | point velocity가 collision distance 변화율로 어떻게 이어지는가? | \(\nabla d=n^T(J_B-J_A)\) | [Collision distance와 gradient](collision-kinematics.md) |
| 2 | 역문제의 안정적인 관절 변화량을 어떻게 유도하는가? | \(\min\|J\Delta q-e\|^2+\lambda^2\|\Delta q\|^2\) | [DLS와 위치 우선 IK 수학](ik-math.md) |
| 3 | DLS 한 step을 반복 IK로 어떻게 구성하는가? | \(\Delta q=J_\lambda^+e+N_\lambda J_R^Te_R\) | [단일 팔 IK](ik.md) |
| 4 | base·lift·양팔과 safety constraint를 어떻게 함께 푸는가? | bounded weighted least-squares + CBF | [전신 IK와 충돌 회피](whole_body_ik.md) |
| 5 | 팔 관절 목표를 실제 torque로 어떻게 바꾸는가? | \(\tau=h+K_pe-K_d\dot q\) | [팔 토크 제어](arm_control.md) |
| 6 | base twist를 세 wheel module 명령으로 어떻게 바꾸는가? | \(v_i=v+\omega\times r_i\) | [모바일 스워브 제어](base_teleop.md) |
| 7 | 두 synergy와 접촉력으로 파지를 어떻게 명령·판정하는가? | \(u=b+c_g g+c_t t\) | [손 파지와 접촉 판정](grasp.md) |

### 공통 설명 규칙

각 알고리즘 문서는 다음 순서를 지킨다.

1. **문제와 경계**: 무엇을 입력받고 무엇을 반환하며, 하지 않는 일을 먼저 밝힌다.
2. **기호와 좌표계**: 수식 전에 벡터의 크기, frame, 단위를 정의한다.
3. **수식 유도**: 가정에서 목적함수 또는 기하 관계를 세우고 결과식까지 전개한다.
4. **코드 대응**: 수식의 각 항을 실제 파일·함수·변수 이름과 연결한다.
5. **실행 흐름**: 한 frame 또는 한 iteration에서 호출되는 순서를 보여준다.
6. **검증**: 어떤 테스트가 수식의 어떤 주장을 확인하는지 명시한다.

근사식, 물리 모델의 한계, 경험적으로 정한 gain은 증명된 기하식과 구분해서 표시한다.

### 기반 지식은 여기서 보충한다

알고리즘 문서에서 같은 배경 설명을 반복하지 않는다. 낯선 개념이 나오면 아래의 이미
설명된 페이지로 돌아간 뒤 원래 학습 순서로 복귀한다.

| 필요한 기반 지식 | 부연 설명 |
|---|---|
| `MjModel`, `MjData`, `qpos`, `qvel`, `ctrl`, contact | [MuJoCo 기본 용어](00-basics.md), [Part 2 — MuJoCo model과 data](ros2/02-mujoco-model-data.md) |
| body–joint–site 관계와 조상 경로 | [Kinematic Tree](kinematic-tree.md) |
| 회전행렬, Rodrigues 식, geometric Jacobian | [FK와 Jacobian](forward-kinematics.md) |
| quaternion 곱·역·부호·자세 오차 | [Quaternion과 자세 오차](quaternion-math.md) |
| least-squares, DLS, SVD, null space | [DLS와 위치 우선 IK 수학](ik-math.md) |
| signed distance, gradient, CBF 경계 | [Collision distance와 gradient](collision-kinematics.md), [전신 IK의 collision avoidance](whole_body_ik.md#reactive-collision-avoidance) |
| world/base/startup-anchor 좌표계 | [목표와 좌표 변환](teleop_targets.md), [Part 10 — 좌표계](ros2/10-coordinate-frames.md) |
| actuator, bias force, contact force | [팔 토크 제어](arm_control.md), [손 파지와 접촉 판정](grasp.md) |

## 코드 계층

```mermaid
flowchart TB
    subgraph App["애플리케이션 계층"]
        UI["teleop_ui.py<br>입력 widget"]
        RENDER["teleop_render.py<br>scene · camera · gizmo"]
        APP["teleop_app.py<br>조립 · 명령 선택 · frame loop"]
        TARGETS["teleop_targets.py<br>target 상태 · 좌표 변환"]
    end

    subgraph Control["제어 알고리즘"]
        WBIK["whole_body_ik.py<br>bounded IK · CBF"]
        BASE["base_teleop.py<br>body twist · swerve"]
        ARM["arm_control.py<br>팔 torque"]
        GRASP["grasp.py<br>손가락 synergy · contact"]
        IK["ik.py<br>단일 팔 IK 호환 이름"]
    end

    KIN["kinematics 모듈군<br>math · tree · solver · collision"]
    PHYS[("MuJoCo model/data")]

    UI --> APP
    RENDER <--> APP
    APP <--> TARGETS
    APP --> WBIK
    APP --> BASE
    APP --> ARM
    APP --> GRASP
    WBIK --> KIN
    IK --> KIN
    BASE --> PHYS
    ARM --> PHYS
    GRASP --> PHYS
    APP <--> PHYS
    PHYS --> RENDER
```

의존 방향의 핵심은 `teleop_app.py`가 조립을 담당하고, 계산 모듈은 UI나 renderer를
알지 않는다는 점이다. `kinematics.py`는 단일 팔 IK와 전신 IK가 함께 사용하는 가장
낮은 수학 계층이다.

## 수정 목적별 경로 { #change-paths }

| 수정 목적 | 먼저 볼 문서 | 함께 볼 문서 | 최소 회귀 |
|---|---|---|---|
| 손 pose/Jacobian | [FK와 Jacobian](forward-kinematics.md) | [Kinematic Tree](kinematic-tree.md), [Quaternion](quaternion-math.md) | Phase 3, Whole-body |
| 단일 팔 IK | [단일 팔 IK](ik.md) | [DLS와 위치 우선 IK 수학](ik-math.md), [기구학](kinematics.md) | Phase 3, 4 |
| 전신 IK·관절 한계·충돌 | [전신 IK와 충돌 회피](whole_body_ik.md) | [목표와 좌표 변환](teleop_targets.md) | Whole-body, Phase 6 |
| 팔 torque | [팔 토크 제어](arm_control.md) | [앱 조립](teleop_app.md) | Phase 3, 4 |
| 바퀴·조향·수동 주행 | [모바일 스워브 제어](base_teleop.md) | [앱 조립](teleop_app.md) | Phase 5, Whole-body |
| 손가락·파지 판정 | [손 파지와 접촉 판정](grasp.md) | [MuJoCo 기본 용어](00-basics.md) | Phase 1, 2 |
| target·marker·좌표계 | [목표와 좌표 변환](teleop_targets.md) | [동작 원리](../concepts.md) | Phase 6 |
| 패널·입력 | [UI 패널](teleop_ui.md) | [앱 조립](teleop_app.md) | Phase 6 |
| 카메라·gizmo·overlay | [렌더링과 Gizmo](teleop_render.md) | [목표와 좌표 변환](teleop_targets.md) | Phase 6 |

## 소스 파일 책임

| 파일 | 한 문장 책임 | 주요 쓰기 대상 |
|---|---|---|
| `teleop_app.py` | 모듈을 초기화하고 frame별 최종 명령을 선택 | app 상태, `data.ctrl`, physics step |
| `teleop_ui.py` | ImGui 입력을 target과 mode 상태로 변환 | app target/mode |
| `teleop_render.py` | scene, camera, gizmo, collision overlay 렌더링 | render state, gizmo target |
| `teleop_targets.py` | UI 값과 world pose를 왕복 변환 | target/marker state |
| `kinematics.py` | 단일 팔 IK와 기존 공개 API 진입점 | 트리·충돌 세부 구현 없음 |
| `kinematic_tree.py` | MJCF 트리, FK와 Jacobian | live data 접근 없음 |
| `kinematics_math.py` | 회전 행렬과 쿼터니언 수학 | 모델·solver 상태 없음 |
| `collision_kinematics.py` | signed-distance gradient | target/solver 정책 없음 |
| `bimanual_kinematics.py` | rigid-grasp 상대 pose와 Jacobian 계산 | actuator·solver 상태 없음 |
| `bounded_optimization.py` | box BVLS와 soft barrier 계산 | robot model 상태 없음 |
| `whole_body_ik.py` | WBIK task·bound·상태를 조립하고 command 계산 | 반환 command만 |
| `base_teleop.py` | body twist를 steer/drive command로 변환 | controller 내부 상태 |
| `arm_control.py` | 목표 관절각을 torque로 변환 | arm `data.ctrl` |
| `grasp.py` | synergy를 finger command로 바꾸고 contact force 판정 | finger `data.ctrl` |
| `ik.py` | 기존 `InverseKinematics` import를 공용 `KinematicsSolver`에 연결 | 없음 |
| `mj_util.py` | joint에서 actuator를 찾는 공용 MuJoCo helper | 없음 |

파일을 찾은 다음에는 [API 치트시트](cheatsheet.md)의 **상황별 첫 함수** 표를 사용한다.
그 표는 직접 호출해도 되는 진입점, 반환값, 다음 단계와 상세 문서를 함께 적는다.
이름이 `_`로 시작하는 함수는 모듈 내부 구현이므로 새 호출부에서는 공개 함수나
`TeleopApp`의 공개 메서드를 우선한다.

## ROS2 관점으로 읽기 { #ros2-reading }

ROS2/Gazebo 경험이 있다면 아래 Part에서 익숙한 node, topic, tf2, MoveIt,
`ros2_control` 개념을 현재 MuJoCo 단일 프로세스 구조와 대응해 볼 수 있다. 별도의
시스템이 아니라 위 코드 계층을 다른 관점으로 설명하는 심화 트랙이다.

| 목적 | 권장 Part |
|---|---|
| ROS2와 전체 구조 차이 | Part 1 → 2 → 4 |
| 제어 알고리즘 | Part 5 → 6 → 7 → 8 |
| 3D 조작과 좌표계 | Part 9 → 10 |
| 검증과 유지보수 | Part 11 → 13 → 14 |
| 설치와 직접 실행 | Part 12 |

### 시작과 구조

| 페이지 | 내용 |
|---|---|
| [Part 1 — ROS2와 개념 지도](ros2/01-concepts.md) | node·topic·tf·controller와 현재 구조 비교 |
| [Part 2 — MuJoCo model과 data](ros2/02-mujoco-model-data.md) | MJCF, actuator, contact, 물리 상태 |
| [Part 3 — 프로젝트 정체성](ros2/03-project-identity.md) | 목표, 불변식, Phase 이력 |
| [Part 4 — 런타임 아키텍처](ros2/04-runtime-architecture.md) | 파일 지도와 한 frame의 실행 순서 |

### 제어 알고리즘

| 페이지 | 내용 |
|---|---|
| [Part 5 — 손 제어](ros2/05-hand-control.md) | grasp synergy, 관절 보간, 접촉 판정 |
| [Part 6 — 전신 IK와 단일 팔 DLS IK](ros2/06-inverse-kinematics.md) | bounded WBIK, legacy DLS, task priority와 회전 오차 |
| [Part 7 — 팔 토크 제어](ros2/07-arm-torque-control.md) | PD와 bias force feedforward |
| [Part 8 — 모바일 베이스](ros2/08-mobile-base.md) | 스워브 역기구학과 feedback 제어 |

### 조작과 좌표계 { #coordinate-reading }

| 페이지 | 내용 |
|---|---|
| [Part 9 — 3D 텔레오퍼레이션 UI](ros2/09-teleoperation-ui.md) | MoveL, bimanual 상태, gizmo |
| [Part 10 — 좌표계](ros2/10-coordinate-frames.md) | startup anchor, world target, 변환 함수 |

### 검증과 참고

| 페이지 | 내용 |
|---|---|
| [Part 11 — 테스트와 검증](ros2/11-testing.md) | Phase gate와 release 전략 |
| [Part 12 — 직접 실행](ros2/12-running.md) | 설치와 실행 명령 |
| [Part 13 — 버그 사례집](ros2/13-bug-cases.md) | 실제 결함과 일반화된 교훈 |
| [Part 14 — 용어와 개념 찾아보기](ros2/14-glossary.md) | 익숙한 용어에서 현재 구현으로 이동 |

이 프로젝트는 ROS2 node가 아니라 `python3 src/teleop_app.py`로 실행되는 단일
프로세스 프로그램이다. 입력이 target을 갱신하고, whole-body IK와 actuator
controller가 `data.ctrl`을 만든 다음 MuJoCo physics와 rendering을 순서대로 수행한다.

```text
입력 → target 갱신 → whole-body IK
    → 팔·손·바퀴 actuator command → mj_step → rendering
```

DLS, SVD gain, null-space projector의 수식은 중복 전개하지 않고
[DLS와 위치 우선 IK 수학](ik-math.md)에 한 흐름으로 정리한다.

## 완독 후 수학 이해 범위

아래는 이 저장소가 **직접 구현한 수학**의 완주 점검표다. 각 행의 질문에 식과 실제
함수 이름을 함께 답할 수 있으면 해당 영역을 이해한 것이다.

| 영역 | 완주 질문 | 상세 문서 | 상태 |
|---|---|---|---|
| target 좌표계 | local RPY/offset이 어떻게 world pose가 되는가? | [목표와 좌표 변환](teleop_targets.md) | 구현 범위 설명 완료 |
| tree와 FK | MJCF 경로를 따라 site pose를 어떻게 합성하는가? | [Tree](kinematic-tree.md), [FK](forward-kinematics.md) | 구현 범위 설명 완료 |
| 자세와 Jacobian | quaternion 오차와 6×N Jacobian의 frame은 왜 일치하는가? | [Quaternion](quaternion-math.md), [FK](forward-kinematics.md) | 구현 범위 설명 완료 |
| 단일 팔 IK | DLS normal equation, SVD gain, null-space 항이 코드의 어느 줄이 되는가? | [DLS 수학](ik-math.md), [단일 팔 IK](ik.md) | 구현 범위 설명 완료 |
| 전신 IK | weighted task, box bound, BVLS, CBF, rigid-grasp 항이 어떻게 한 문제에 들어가는가? | [전신 IK](whole_body_ik.md) | 구현 범위 설명 완료 |
| 충돌 거리 | 최근접점 속도에서 \(\dot d=\nabla d\dot q\)를 어떻게 얻는가? | [Collision distance](collision-kinematics.md) | 구현 범위 설명 완료 |
| 팔·베이스·손 | IK 출력을 torque, wheel command, finger command로 어떻게 변환하는가? | [팔](arm_control.md), [스워브](base_teleop.md), [파지](grasp.md) | 구현 범위 설명 완료 |

따라서 문서를 모두 읽으면 **현재 시스템이 사용하는 기하·최적화·제어 수학은
수식에서 코드까지 이해할 수 있는 구조**다. 다만 MuJoCo 접촉 solver의 내부 유도,
일반적인 강체동역학 전체 과정, 전역 motion planning, 임의 로봇을 위한 제어 안정성
증명은 이 저장소가 직접 구현하지 않으므로 범위 밖이다. 해당 항목까지 설명했다고
오해하지 않는 것이 중요하다.

## 반드시 지킬 불변식

- 초기화와 자유물체 reset 외에는 live robot `data.qpos`를 직접 덮어쓰지 않는다.
  시작 시 `_disable_legacy_box_asset()`이 사용하지 않는 box를 비활성화하는 것은
  can-only workflow를 위한 명시적 초기화 예외다.
- UI와 gizmo는 target/state만 바꾸고 actuator command를 직접 만들지 않는다.
- quaternion을 정규화하고 orientation error와 rotational Jacobian의 frame을 맞춘다.
- FK arm과 Whole-body OFF DOF는 weight가 아니라 solver bound로 정확히 고정한다.
- keyboard와 WBIK base command는 같은 `SwerveDrive` 경로를 사용한다.
- wheel-floor, finger-object 같은 의도된 contact는 collision CBF에서 제외한다.
- 좌표계나 mode 전환은 target의 world pose를 보존해야 한다.

## 개발 전후 체크

1. [개발 체크리스트](pitfalls.md)에서 MuJoCo/NumPy 함정을 확인한다.
2. 변경 파일과 직접 연결된 최소 테스트를 먼저 실행한다.
3. 최종적으로 [테스트와 검증](../testing.md)의 전체 suite를 실행한다.
4. 문서를 바꿨다면 `mkdocs build --strict`도 실행한다.

짧은 함수 서명과 기본값은 [API 치트시트](cheatsheet.md)에서 찾을 수 있다.
