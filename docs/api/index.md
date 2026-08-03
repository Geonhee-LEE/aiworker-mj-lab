# API 레퍼런스

`ffw_sh5_grasp`의 공개 Python 인터페이스를 기능별로 찾는 독립 참고서다. 시스템의
수학과 설계 배경을 순서대로 배우려면 [시스템 이해와 개발](../guide/index.md)을,
지금 호출할 함수의 입력·반환값을 찾으려면 이 절을 사용한다.

[빠른 API 찾기](quick-reference.md){ .md-button .md-button--primary }
[패키지 구조 보기](#api-architecture){ .md-button }

## API 구조 { #api-architecture }

| 계층 | Python 모듈 | 주요 기능 | 대표 사용 상황 |
|---|---|---|---|
| 설정 | `ffw_sh5_grasp.config` | 기본·사용자 YAML 병합과 형식 검증 | 실행 전 파라미터 선택 |
| 애플리케이션 | `ffw_sh5_grasp.application` | 앱 생명주기, 모드 전환, target 좌표 변환 | 텔레옵 실행·UI 목표 처리 |
| 기구학 | `ffw_sh5_grasp.kinematics` | 회전 수학, Tree, FK/Jacobian, 단일 팔 IK, 충돌 거리 | pose·Jacobian·IK 계산 |
| 제어 | `ffw_sh5_grasp.control` | 전신 IK, BVLS, 팔 토크, 스워브, 파지 | 물리 actuator 명령 생성 |
| 시각화 | `ffw_sh5_grasp.visualization` | ImGui UI, MuJoCo 장면, 카메라, Gizmo | 조작 화면과 진단 도구 |

```text
ffw_sh5_grasp
├── config                  YAML 설정 로딩과 검증
├── application
│   ├── teleop              앱 조립과 frame loop
│   └── targets             UI target ↔ world pose
├── kinematics
│   ├── rotations           Quaternion·회전행렬 공용 수학
│   ├── tasks               공통 pose 오차·Cartesian 속도 명령
│   ├── tree                MJCF Tree·FK·Jacobian
│   ├── solver              단일 팔 DLS IK
│   └── collision           거리와 gradient
├── control
│   ├── whole_body          18-DOF 명령 조립
│   ├── optimization        BVLS와 collision soft barrier
│   ├── bimanual            양손 rigid-grasp task
│   ├── arm                 팔 토크 제어
│   ├── base                스워브 제어
│   └── grasp               손가락 synergy·접촉 판정
└── visualization
    ├── ui                  ImGui 패널
    └── render              장면·카메라·Gizmo
```

## 상세 모듈 문서

### 설정

[설정 API](config.md)는 `load_settings()`와 `Settings`의 타입별 조회·검증 규칙을
설명한다. 사용자 YAML을 프로그램에서 직접 선택할 때 시작한다.

### 애플리케이션

[애플리케이션 API](application.md)는 `TeleopApp` 생명주기와 손·가상 물체 target의
좌표 변환 함수를 다룬다. UI나 외부 입력을 IK 목표로 연결할 때 사용한다.

### 기구학

[기구학 API](kinematics.md)는 회전 유틸리티, `KinematicTree`,
`KinematicsSolver`, 충돌 거리 자료형과 함수를 다룬다. 물리 상태를 수정하지 않는
pose·Jacobian·IK 계산이 필요할 때 사용한다.

### 제어

[제어 API](control.md)는 `WholeBodyIK`, `ArmTorqueController`, `SwerveDrive`, 파지
함수와 bounded 최적화 인터페이스를 다룬다. 기구학 결과를 실제 명령으로 만들 때
사용한다.

### 시각화

[시각화 API](visualization.md)는 렌더 생명주기, 카메라, Gizmo와 UI 진입점을 다룬다.
새 패널이나 진단 표시를 연결할 때 사용한다.

## 공개 API 규칙

- Quaternion 순서는 MuJoCo와 같은 `(w, x, y, z)`다.
- 위치 단위는 m, 각도는 별도 표기가 없으면 rad다. 함수명에 `_deg`가 있으면 degree다.
- `KinematicTree`와 IK solver는 입력 `qpos`와 live `MjData`를 직접 수정하지 않는다.
- `WholeBodyIK.solve()`는 명령을 반환하고, 팔·손 controller의 `apply*()`가
  `data.ctrl`을 기록한다.
- 이름이 `_`로 시작하는 함수와 메서드는 내부 구현이며 호환성을 보장하지 않는다.

!!! note "기존 진입점"
    `src/teleop_app.py`, `src/kinematics.py`, `src/ik.py`는 기존 실행 명령과 import를
    위한 얇은 호환 계층이다. 새 코드는 `ffw_sh5_grasp` 패키지에서 import한다.
