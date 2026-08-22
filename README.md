# aiworker-mj-lab

ROBOTIS AIWORKER(FFW-SH5)를 MuJoCo에서 연구하기 위한 저장소입니다. 현재는 양팔
텔레오퍼레이션, 직접 구현한 FK/Jacobian과 differential IK, 전신 제어, 스워브 주행,
접촉 기반 파지와 충돌 회피를 하나의 Python 앱에서 실행합니다.

ALOHA-style 모방학습 경로는 별도의 arm-only 계층으로 제공됩니다. 세 policy camera와
joint/task-space ACT, HDF5 record/replay, 선행 앙상블(PTE), 색상 분류 평가와 Rerun
분석을 포함하며 기존 Whole-body 텔레옵 기능과 분리되어 있습니다.

[문서 사이트](https://ggh-png.github.io/aiworker-mj-lab/) ·
[빠른 시작](docs/getting-started.md) ·
[3.0.0 릴리스](https://github.com/ggh-png/aiworker-mj-lab/releases/tag/v3.0.0)

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
| 모방학습 | 4색 can sorting, joint/task ACT, PTE, 2,000-rollout 평가 |
| 검증 | Ruff, pytest, Phase 0–6, Whole-body 및 strict 문서 빌드 |

로봇 관절이나 베이스 pose를 순간 이동시켜 결과를 만들지 않습니다. IK와 controller가
명령을 계산하고, 실제 actuator와 MuJoCo contact가 다음 상태를 만듭니다. ROS, MoveIt,
Pinocchio, FCL, OSQP는 런타임 의존성이 아닙니다.

## 빠른 실행

Linux 데스크톱과 OpenGL 화면 세션이 필요합니다.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install mujoco numpy glfw imgui-bundle pyyaml h5py torch pillow imageio rerun-sdk wandb
python3 src/teleop_app.py
```

환경 번호를 명시하면 기존 단일 상자와 색상 분류 환경을 구분해 실행할 수 있습니다.
색상 분류 환경에서는 `R`을 누를 때마다 캔 색과 좌·우 상자 색 배치를 서로 독립적으로
무작위 선택합니다.

```bash
python3 src/teleop_app.py --env 0  # 기존 초록 캔 -> 파랑 상자
python3 src/teleop_app.py --env 1  # 초록/빨강/주황/파랑 캔 색상 분류
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
python -m pip install -r requirements-dev.txt -r requirements-docs.txt
python -m ruff check src scripts tests
MUJOCO_GL=egl python -m pytest -q
python -m mkdocs build --strict
```

## Arm-only ACT 파이프라인

첫 시나리오는 무작위 위치의 캔을 오른팔로 고정된 파란 목표 상자에 넣는 작업입니다.
이 경로에서는 base/lift/head를 고정하고 Whole-body IK를 사용하지 않습니다. 왼팔은
상자와 간섭하지 않는 palm-up 자세로 고정되며, 상자 바닥·네 벽은 실제 contact를
만듭니다.

```bash
# recorder와 live Rerun Viewer가 함께 열린다.
# R: task pose + random can reset, Q: grab/release, SPACE: record
python3 src/il.py record --task-name can_to_box

# 네 색 전체를 무작위로 수집
python3 src/il.py record --task-name can_color_sort

# 부족한 주황/파랑 캔만 수집 (기존 can_color_sort dataset에 이어서 저장)
python3 src/il.py record --task-name can_color_sort \
  --variant orange --variant blue

python3 src/il.py rerun \
  --episode datasets/can_to_box/episode_000000.hdf5
python3 src/il.py replay \
  --episode datasets/can_to_box/episode_000000.hdf5

python3 src/il.py validate \
  --dataset-dir datasets/can_to_box \
  --camera cam_high --camera cam_right_wrist

python3 src/il.py train --config config/imitation/act.yaml
python3 src/il.py evaluate \
  --checkpoint outputs/act/can_to_box_act_v2/checkpoints/policy_best.ckpt

# Control Center의 ACT Policy 탭에서 outputs/act 아래 모델을 선택한다.
python3 src/teleop_app.py

# Modular joint/task checkpoint는 metadata로 입력 표현을 자동 판별한다.
python3 src/teleop_app.py --env 1 \
  --policy-checkpoint outputs/act_modular/can_color_sort_act_task/checkpoints/policy_best.ckpt \
  --policy-representation auto \
  --policy-ik-speed-scale 3.0 \
  --policy-pte-steps 0 \
  --policy-rerun
```

ACT Policy 패널에서 `AUTO`/`JOINT`/`TASK`로 checkpoint 목록을 필터링하고,
`PTE future steps`를 실행 중에도 바꿀 수 있다. `0`은 기존 ACT temporal ensemble이고
양수는 같은 checkpoint가 예측한 해당 step만큼 미래 action을 현재 실행한다.
정책 제어는 25 Hz로 유지하면서 Rerun은 기본 5 Hz와 JPEG 압축으로 기록해 Viewer가
제어 loop를 막지 않게 한다. 기록 주기는 `--policy-rerun-hz`로 바꿀 수 있다.

W&B 학습 대시보드를 사용하려면 한 번 로그인한 후 학습한다. 기본 프로젝트 이름과
활성화 여부는 `config/imitation/act.yaml`의 `wandb`에서 변경한다.

```bash
wandb login
python3 src/il.py train --config config/imitation/act.yaml
```

대규모 수집 전에 성공 episode 하나로 overfit gate를 통과시키는 것을 기본 절차로
삼습니다. 자세한 schema와 산출물은 [모방학습 가이드](docs/guide/imitation-sim2real.md),
논문 구조와 FFW-SH5 적응점은 [ACT 구현 대응표](docs/guide/act-implementation.md)에
정리되어 있습니다.

전체 Phase 실행 명령과 각 gate의 의미는 [테스트와 검증](docs/testing.md)에 있습니다.

## Hugging Face 배포

색상 분류 HDF5 데이터셋과 네 ACT 정책을 서로 다른 Hub 저장소로 배포합니다.
배포 전 manifest를 생성하면서 episode schema, 카메라, 성공 여부와 모델 산출물을
검증합니다. `policy_last.ckpt`, Rerun과 W&B 로그는 배포 대상에서 제외됩니다.

- [FFW-SH5 Can Color Sort 데이터셋](https://huggingface.co/datasets/ggh-png/ffw-sh5-can-color-sort)
- [FFW-SH5 ACT Color Sort 정책](https://huggingface.co/ggh-png/ffw-sh5-act-color-sort)

```bash
python -m pip install -r requirements-huggingface.txt
hf auth login

# 파일 목록·용량·metadata만 점검하며 업로드하지 않는다.
python3 scripts/publish_huggingface.py \
  --dataset-repo-id ggh-png/ffw-sh5-can-color-sort \
  --model-repo-id ggh-png/ffw-sh5-act-color-sort \
  --dry-run

# 기본값은 private 저장소다. 검증 후 공개 저장소로 업로드한다.
HF_XET_HIGH_PERFORMANCE=1 python3 scripts/publish_huggingface.py \
  --dataset-repo-id ggh-png/ffw-sh5-can-color-sort \
  --model-repo-id ggh-png/ffw-sh5-act-color-sort \
  --public
```

Hub에서 받는 명령과 파일 구성은 [Hugging Face 배포 문서](docs/huggingface.md), 카드
원본은 `huggingface/dataset`과 `huggingface/model`에 있습니다.

## 코드 구조

```text
config/default.yaml                 실행·제어 기본 설정
models/                             MuJoCo 모델
src/teleop_app.py                   teleop 실행 진입점
src/il.py                           모방학습 통합 명령 dispatcher
src/ffw_sh5_grasp/cli/              IL command별 argument parser
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
