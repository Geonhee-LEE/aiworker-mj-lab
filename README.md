# ffw-sh5-grasp

ROBOTIS FFW-SH5 양팔 모바일 로봇의 MuJoCo 물리 기반 텔레오퍼레이션 프로젝트입니다.
ROS 없이 손 목표, whole-body/arm-only IK, 실제 스워브 바퀴, 양손 grasp와 reactive
collision avoidance를 하나의 Python 앱에서 실행합니다.

## 데모 영상

| Whole-body Control | Whole-body 미사용 (Arm-only) |
|---|---|
| [![Whole-body Control 데모](https://img.youtube.com/vi/MzO1GpUfCd8/hqdefault.jpg)](https://www.youtube.com/watch?v=MzO1GpUfCd8&list=PLWyQPsEn5Atg) | [![Whole-body 미사용 데모](https://img.youtube.com/vi/2LV_RsAGdz8/hqdefault.jpg)](https://www.youtube.com/watch?v=2LV_RsAGdz8&list=PLWyQPsEn5Atg&index=2) |
| base·lift·양팔이 함께 목표를 추종하는 버전 | base·lift 자동 참여를 끈 버전 |
| [알고리즘 상세 구현: 전신 IK와 충돌 회피](docs/guide/whole_body_ik.md) | [알고리즘 상세 구현: Arm-only hard gate](docs/guide/whole_body_ik.md#whole-body-modes) |

[문서 사이트](https://ggh-png.github.io/ffw-sh5-grasp/) ·
[2.0.0 릴리스](https://github.com/ggh-png/ffw-sh5-grasp/releases/tag/v2.0.0)

## 먼저 읽을 문서

| 목적 | 문서 |
|---|---|
| 설치부터 첫 동작까지 | [빠른 시작](docs/getting-started.md) |
| 버튼과 키 전체 목록 | [화면과 조작](docs/run.md) |
| MoveL/IK/Whole-body 조합 | [모드 선택](docs/control-modes.md) |
| 느림·잔류 주행·collision 진단 | [문제 해결](docs/troubleshooting.md) |
| 구조와 알고리즘 이해·수정 | [시스템 이해와 개발 가이드](docs/guide/index.md) |
| 테스트 근거 | [테스트와 검증](docs/testing.md) |

## 핵심 기능

- FFW-SH5 전신: 양팔 7DOF×2, HX5-D20 양손, 리프트, 헤드, 3모듈 스워브 베이스
- Whole-body IK: base x/y/yaw + lift + 양팔 14축 bounded differential solve
- Custom kinematics: MJCF body–joint–site 트리에서 FK와 geometric Jacobian 직접 계산
- **Whole-body ON/OFF**: world target을 보존하며 전신 IK와 arm-only hard gate 전환
- Bimanual MoveL: virtual object와 captured rigid-grasp relative-pose constraint
- Collision avoidance: 팔-팔·팔-몸체·팔/손-table 3 cm 감시, 1 cm safe-distance CBF
- 실제 mobile physics: steer/drive actuator와 wheel-ground friction으로만 base 이동
- Contact grasp: 캔을 로봇에 붙이지 않고 finger contact force와 마찰로 파지
- ROS-free: MoveIt, Pinocchio, FCL, OSQP 없이 NumPy + MuJoCo 알고리즘 구현
- Compact multi-viewport UI: Control Center와 Diagnostics를 별도 OS 창·내부 탭으로 제공
- Headless regression: Phase 0–6, randomized WBIK, collision gradient, 실제 바퀴 추종

## 빠른 실행

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install mujoco numpy glfw imgui-bundle pyyaml
python3 src/teleop_app.py
```

보조 이미지/mesh 스크립트에는 `pillow`, `trimesh`가 추가로 필요합니다.

실행·IK·제어·스워브·파지·UI 튜닝값은 한국어 주석이 포함된
[`config/default.yaml`](config/default.yaml)에 있습니다. 일부 값만 바꾼 사용자
파일은 다음처럼 적용합니다.

```bash
python3 src/teleop_app.py --config config/local.yaml
```

자세한 병합·검증 규칙은 [YAML 파라미터 설정](docs/configuration.md)을 참고합니다.

## 가장 중요한 조작

| 입력 | 기능 |
|---|---|
| `Up` / `Down` | base 전진 / 후진 |
| `Left` / `Right` | base yaw |
| `[` / `]` | base strafe |
| `Q` / `E` | lift 하강 / 상승 |
| `R` | can reset |
| `G` | 실제 contact 표시 |
| `V` | collision CBF geometry/최근접점 표시 |
| `C` | camera preset |

`Lift / Utilities`의 **Whole-body Control** 버튼:

- `ON`: base, lift, IK 팔이 손 목표를 함께 추종합니다.
- `OFF (arm-only)`: base/lift IK 속도를 정확히 0으로 고정하고 팔만 풉니다.
- OFF에서도 keyboard base와 manual lift는 사용할 수 있습니다.
- 전환 시 hand/virtual-object world target과 marker 위치를 보존하고 이전 base twist를
  초기화합니다.

## 테스트

핵심 회귀:

```bash
python3 tests/test_phase_6.py
python3 tests/test_whole_body.py
```

전체 회귀:

```bash
for p in 0 1 2 3 4 5 6; do
  python3 "tests/test_phase_${p}.py"
done
python3 tests/test_config.py
python3 tests/test_whole_body.py
```

문서:

```bash
python -m pip install mkdocs mkdocs-material
mkdocs build --strict
mkdocs serve
```

## 코드 지도

```text
config/
└── default.yaml                  # 한국어 주석 실행·알고리즘 기본 설정
src/
├── teleop_app.py                 # 기존 실행 명령용 launcher
├── kinematics.py                 # 기존 import 호환 facade
├── ik.py                         # InverseKinematics 호환 facade
└── ffw_sh5_grasp/
    ├── config.py                 # YAML 병합·검증과 설정 조회
    ├── application/
    │   ├── teleop.py             # app 조립, command 중재, frame/physics loop
    │   └── targets.py            # target 좌표 변환, marker, Bimanual state
    ├── visualization/
    │   ├── ui.py                 # ImGui widget와 mode/target 입력
    │   └── render.py             # 장면·카메라·ImGuizmo·충돌 오버레이
    ├── kinematics/
    │   ├── solver.py             # 단일 site FK/IK 공개 진입점
    │   ├── legacy.py             # 기존 InverseKinematics 이름의 호환 계층
    │   ├── tree.py               # MJCF tree와 FK/Jacobian
    │   ├── rotations.py          # 회전행렬·quaternion 수학
    │   └── collision.py          # 충돌 거리 기울기
    ├── control/
    │   ├── whole_body.py         # WBIK task·bound·command 조립
    │   ├── optimization.py       # 명시적 box-QP와 soft barrier
    │   ├── bimanual.py           # rigid-grasp 상대 task
    │   ├── arm.py                # 팔 torque 제어
    │   ├── base.py               # BodyTwist와 swerve 제어
    │   └── grasp.py              # finger synergy와 접촉 판정
    ├── mujoco_utils.py           # 공용 MuJoCo name/actuator helper
    └── paths.py                  # 저장소와 모델 경로
```

코드를 처음 읽는다면 [시스템 이해와 개발 가이드](docs/guide/index.md)의 목적별
읽기 경로에서 시작하는 것이 가장 짧습니다.
