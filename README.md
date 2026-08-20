# aiworker-mj-lab

ROBOTIS AIWORKER(FFW-SH5)를 MuJoCo에서 연구하기 위한 저장소입니다. 현재는 양팔
텔레오퍼레이션, 직접 구현한 FK/Jacobian과 differential IK, 전신 제어, 스워브 주행,
접촉 기반 파지와 충돌 회피를 하나의 Python 앱에서 실행합니다.

ALOHA-style 모방학습 경로는 별도의 arm-only 계층으로 제공됩니다. 세 policy camera,
16D joint action, HDF5 record/replay, ACT 학습·평가와 Rerun 분석을 포함하며 기존
Whole-body 텔레옵 기능과 분리되어 있습니다.

[문서 사이트](https://ggh-png.github.io/aiworker-mj-lab/) ·
[빠른 시작](docs/getting-started.md) ·
[2.0.0 릴리스](https://github.com/ggh-png/aiworker-mj-lab/releases/tag/v2.0.0)

## 데모

| Whole-body Control | Arm-only |
|---|---|
| [![Whole-body Control 데모](https://img.youtube.com/vi/AXAByoi5CxU/hqdefault.jpg)](https://www.youtube.com/watch?v=AXAByoi5CxU) | [![Arm-only 데모](https://img.youtube.com/vi/2LV_RsAGdz8/hqdefault.jpg)](https://www.youtube.com/watch?v=2LV_RsAGdz8&list=PLWyQPsEn5Atg&index=2) |
| base·lift·양팔이 함께 목표를 추종 | base·lift의 자동 참여를 끄고 팔만 추종 |

두 모드의 계산 과정과 실제 코드 흐름은 [전신 IK와 충돌 회피](docs/guide/whole_body_ik.md)에
정리되어 있습니다.

## 현재 구현

| 영역 | 내용 |
|---|---|
| 목표 입력 | 손의 world XYZ/RPY, marker jog, 3D Gizmo, 양손 virtual object |
| 기구학 | MJCF tree 기반 FK와 geometric Jacobian, quaternion 자세 오차 |
| IK | Pseudoinverse, DLS, box-QP, Whole-body/Arm-only 전환 |
| 모바일 제어 | base x/y/yaw와 lift를 포함한 전신 IK, 3모듈 steer/drive actuator |
| 파지 | 양손 relative-pose constraint, finger contact force와 마찰 기반 파지 |
| 충돌 대응 | 팔-팔·팔-몸체·팔/손-table 거리 gradient와 velocity CBF |
| 검증 | Phase 0–6, Whole-body 및 YAML 설정 headless 회귀 |

로봇 관절이나 베이스 pose를 순간 이동시켜 결과를 만들지 않습니다. IK와 controller가
명령을 계산하고, 실제 actuator와 MuJoCo contact가 다음 상태를 만듭니다. ROS, MoveIt,
Pinocchio, FCL, OSQP는 런타임 의존성이 아닙니다.

## 빠른 실행

Linux 데스크톱과 OpenGL 화면 세션이 필요합니다.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install mujoco numpy glfw imgui-bundle pyyaml h5py torch pillow imageio rerun-sdk
python3 src/teleop_app.py
```

일부 설정만 덮어쓴 YAML 파일은 다음처럼 적용합니다.

```bash
python3 src/teleop_app.py --config config/local.yaml
```

전체 설치·첫 조작은 [빠른 시작](docs/getting-started.md), 설정 병합과 검증 규칙은
[YAML 파라미터 설정](docs/configuration.md)을 참고합니다.

## 주요 조작

| 입력 | 기능 |
|---|---|
| `Up` / `Down` | base 전진 / 후진 |
| `Left` / `Right` | base yaw |
| `[` / `]` | base strafe |
| `Q` / `E` | lift 하강 / 상승 |
| `V` | collision CBF 시각화 |
| `G` | 실제 contact 표시 |

Task Space 숫자 입력, marker, IK/FK, 파지와 진단 패널은 [화면과 조작](docs/run.md)에
정리되어 있습니다.

## 문서

| 목적 | 문서 |
|---|---|
| 제어 모드 선택 | [MoveL, FK, Whole-body 조합](docs/control-modes.md) |
| 구현 흐름 파악 | [시스템 이해와 개발](docs/guide/index.md) |
| 함수와 데이터 확인 | [API 레퍼런스](docs/api/index.md) |
| 이상 동작 진단 | [문제 해결](docs/troubleshooting.md) |
| 회귀 근거 확인 | [테스트와 검증](docs/testing.md) |

## 테스트

핵심 회귀와 문서 검증:

```bash
python3 tests/test_config.py
python3 tests/test_phase_6.py
python3 tests/test_whole_body.py
mkdocs build --strict
```

## Arm-only ACT 파이프라인

첫 시나리오는 무작위 위치의 캔을 오른팔로 고정된 파란 목표 상자에 넣는 작업입니다.
이 경로에서는 base/lift/head를 고정하고 Whole-body IK를 사용하지 않습니다. 왼팔은
상자와 간섭하지 않는 palm-up 자세로 고정되며, 상자 바닥·네 벽은 실제 contact를
만듭니다.

```bash
# R: task pose + random can reset, Q: grab/release, SPACE: record
python3 src/record_episodes.py --task-name can_to_box

python3 src/rerun_episode.py \
  --episode datasets/can_to_box/episode_000000.hdf5
python3 src/replay_episodes.py \
  --episode datasets/can_to_box/episode_000000.hdf5

python3 src/train_act.py --config config/imitation/act.yaml
python3 src/eval_act.py \
  --checkpoint outputs/act/can_to_box_overfit/checkpoints/policy_best.ckpt
```

대규모 수집 전에 성공 episode 하나로 overfit gate를 통과시키는 것을 기본 절차로
삼습니다. 자세한 schema와 산출물은 [모방학습 가이드](docs/guide/imitation-sim2real.md)에
정리되어 있습니다.

전체 Phase 실행 명령과 각 gate의 의미는 [테스트와 검증](docs/testing.md)에 있습니다.

## 코드 구조

```text
config/default.yaml                 실행·제어 기본 설정
models/                             MuJoCo 모델
src/teleop_app.py                   실행 진입점
src/ffw_sh5_grasp/application/      앱 loop, 모델 주소, 상태·명령과 목표 좌표
src/ffw_sh5_grasp/kinematics/       tree, FK/Jacobian, IK와 충돌 거리
src/ffw_sh5_grasp/control/          팔·전신·스워브·양손·파지 제어
src/ffw_sh5_grasp/visualization/    UI, renderer와 진단 표시
src/ffw_sh5_grasp/imitation/        arm-only dataset, ACT, replay와 Rerun
tests/                              headless 회귀와 보조 도구
```

저장소 이름은 확장 범위를 반영해 `aiworker-mj-lab`으로 바꿨지만, 기존 import와 실행
호환성을 위해 Python 패키지명 `ffw_sh5_grasp`는 유지합니다.

## 확장 방향

- ACT 정책의 실물 observation/command adapter와 안전 계층
- 환경 collision check를 사용하는 고전적 경로 탐색과 trajectory 실행
