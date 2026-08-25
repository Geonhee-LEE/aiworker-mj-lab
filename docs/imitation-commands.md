# Imitation Learning Command Reference

이 문서는 can-to-box 오른팔 ACT 워크플로의 모든 실행 명령을 한 곳에 정리한다.
모든 명령은 저장소 루트에서 Python 3.12 가상환경을 활성화한 뒤 실행한다.

```bash
git clone https://github.com/ggh-png/aiworker-mj-lab.git
cd aiworker-mj-lab
```

## Setup

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-imitation.txt
```

W&B를 사용할 때만 `wandb login`을 실행한다. 사용하지 않으면 학습 YAML의
`wandb.enabled`를 `false`로 설정한다. GUI 앱은 Linux 데스크톱 OpenGL 세션에서
실행하고, headless renderer가 필요한 명령은 NVIDIA 환경에서 `MUJOCO_GL=egl`, CI와
OSMesa 환경에서 `MUJOCO_GL=osmesa`를 지정한다.

직접 수집·학습하지 않고 공개 정책과 dataset으로 시작하려면 먼저
[Hugging Face 정책·데이터셋](huggingface.md)을 따른다. 해당 문서에는 고정된 `v3.1.0`
revision 다운로드, checkpoint 실행, HDF5 검증과 재학습 경로 연결이 포함되어 있다.

## Workflow

| 단계 | 명령 | 결과 |
|---|---|---|
| demonstration 기록 | `python3 src/il.py record --task-name can_to_box` | `datasets/can_to_box/episode_*.hdf5` |
| 색상 분류 demonstration 기록 | `python3 src/il.py record --task-name can_color_sort` | `datasets/can_color_sort/episode_*.hdf5` |
| 주황/파랑만 추가 기록 | `python3 src/il.py record --task-name can_color_sort --variant orange --variant blue` | 기존 color-sort dataset에 연속 저장 |
| dataset 검증 | `python3 src/il.py validate --dataset-dir datasets/can_to_box --camera cam_high --camera cam_right_wrist` | schema/alignment 요약 |
| RGB video 확인 | `python3 src/il.py visualize --dataset-dir datasets/can_to_box --episode-idx 0` | episode 옆 `.mp4` |
| Rerun dataset 보기 | `python3 src/il.py rerun --episode datasets/can_to_box/episode_000000.hdf5` | `.rrd` |
| physics replay | `python3 src/il.py replay --dataset-dir datasets/can_to_box --episode-idx 0` | qpos 재현 결과 |
| ACT 학습 | `python3 src/il.py train --config config/imitation/act.yaml` | checkpoint, metric, W&B run |
| Joint/Task ACT 학습 | `python3 src/il.py train --config config/imitation/act_color_sort_joint.yaml` | 표현별 독립 checkpoint |
| closed-loop 평가 | `python3 src/il.py evaluate --checkpoint outputs/act/<run>/checkpoints/policy_best.ckpt` | `evaluation.json` |
| 색상 분류 평가 행렬 | `python3 src/il.py evaluate-color-sort` | 4 정책 × 5 PTE × 100회 CSV/JSON |
| expert/policy 비교 | `python3 src/il.py compare --checkpoint outputs/act/<run>/checkpoints/policy_best.ckpt --episode datasets/can_to_box/episode_000000.hdf5` | comparison `.rrd` |
| ACT Grad-CAM | `python3 src/il.py gradcam --checkpoint outputs/act/<run>/checkpoints/policy_best.ckpt --episode datasets/can_to_box/episode_000000.hdf5` | 카메라별 PNG overlay와 원본 `.npz` |
| interactive policy UI | `python3 src/teleop_app.py` | UI에서 `outputs/act`의 ACT 모델 선택 |

## Record Demonstrations

```bash
python3 src/il.py record \
  --task-name can_to_box \
  --dataset-dir datasets/can_to_box \
  --seed 42
```

`--dataset-dir`을 생략하면 `datasets/<task-name>`을 사용한다. 기본값은 live Rerun을
연다. GUI 없이 기록 UI만 쓸 때는 `--no-live-rerun`, 다른 포트를 쓸 때는
`--rerun-port 9876`을 지정한다.

기록 UI 조작은 `SPACE`(기록 시작/완료), `R`(reset), `BACKSPACE`(폐기), `Q`(오른손
grasp), `E`(오른팔 home)다.

색상별 데이터 수를 보충할 때는 `--variant`를 반복해서 지정한다. 아래 명령은 주황과
파랑만 균등하게 선택하고, 좌우 상자 색 배치는 reset마다 별도로 무작위 선택한다.

```bash
python3 src/il.py record \
  --task-name can_color_sort \
  --variant orange \
  --variant blue
```

## Inspect and Replay Episodes

학습 전에 전체 dataset의 shape, dtype, timestep 정렬, finite 값과 필수 카메라를 확인한다.

```bash
python3 src/il.py validate \
  --dataset-dir datasets/can_to_box \
  --camera cam_high \
  --camera cam_right_wrist
```

두 명령은 모두 명시 경로 또는 dataset/index 조합을 받는다.

```bash
# Explicit path
python3 src/il.py visualize \
  --episode datasets/can_to_box/episode_000000.hdf5 \
  --output outputs/episode_000000.mp4

# Dataset plus index
python3 src/il.py replay \
  --dataset-dir datasets/can_to_box \
  --episode-idx 0 \
  --atol 0.0005 \
  --viewer
```

Rerun 파일을 저장하거나 live stream을 연다.

```bash
python3 src/il.py rerun \
  --episode datasets/can_to_box/episode_000000.hdf5 \
  --output outputs/episode_000000.rrd

python3 src/il.py rerun \
  --episode datasets/can_to_box/episode_000000.hdf5 \
  --live --port 9877
```

## Train ACT

```bash
python3 src/il.py train --config config/imitation/act.yaml
```

기본 설정은 `cam_high`, `cam_right_wrist`, 오른팔 qpos/action 8D를 사용한다.
출력은 `outputs/act/<run_name>/`에 저장되며, `policy_best.ckpt`와
`policy_last.ckpt`, split, normalization stats, CSV/JSONL metrics, Rerun, W&B
metrics를 포함한다.

새 데이터를 추가했다면 새 `run_name`을 지정하고 다시 학습한다. 현재 trainer는
checkpoint resume을 제공하지 않으므로 새 전체 dataset으로 stats와 split을 다시 만든다.

Joint/Task 표현 비교와 150 episode 설정도 같은 `train` 명령을 사용한다. YAML의
`representation: joint|task`만 달라진다.

```bash
python3 src/il.py train \
  --config config/imitation/act_color_sort_joint.yaml
python3 src/il.py train \
  --config config/imitation/act_color_sort_task.yaml
python3 src/il.py train \
  --config config/imitation/act_color_sort_joint_aug150.yaml
python3 src/il.py train \
  --config config/imitation/act_color_sort_task_aug150.yaml
```

같은 seed와 PTE 조건으로 2,000 rollout 계획을 먼저 확인한 뒤 실행한다.

```bash
python3 src/il.py evaluate-color-sort --dry-run
MUJOCO_GL=egl python3 src/il.py evaluate-color-sort --num-episodes 100
```

## Evaluate and Compare

```bash
python3 src/il.py evaluate \
  --checkpoint outputs/act/<run>/checkpoints/policy_best.ckpt \
  --num-episodes 10 \
  --max-steps 500 \
  --seed 1000 \
  --viewer

python3 src/il.py compare \
  --checkpoint outputs/act/<run>/checkpoints/policy_best.ckpt \
  --episode datasets/can_to_box/episode_000000.hdf5 \
  --output outputs/expert_policy_compare.rrd
```

`evaluate`의 `--no-rerun`은 rollout Rerun 기록을 끈다. `--stats`는 checkpoint
옆의 `dataset_stats.pkl` 대신 별도 통계 파일을 사용할 때만 지정한다.

## Explain ACT Predictions with Grad-CAM

ACT는 분류 logit이 아니라 연속 action chunk를 출력한다. 따라서 이 명령의 Grad-CAM은
물체 class가 아니라 **선택한 action 출력에 영향을 준 이미지 영역**을 설명한다. 아래
`--target chunk`는 한 checkpoint 안에서 전체 출력 민감도를 빠르게 확인하는 진단값이며,
Joint와 Task 표현을 같은 물리 의미로 비교하는 target이 아니다.

```bash
python3 src/il.py gradcam \
  --checkpoint outputs/act/<run>/checkpoints/policy_best.ckpt \
  --episode datasets/can_color_sort/episode_000147.hdf5 \
  --frames 0 120 200 280 350 \
  --target chunk \
  --output-dir outputs/analysis/gradcam/<run>_episode147
```

Task-space 정책의 특정 미래 운동을 분석할 수도 있다. Task action index는
`0:3=world xyz`, `3:7=wxyz quaternion`, `7=grasp`다. 예시는 현재 frame에서
89-step 뒤의 음의 world-y 이동을 설명한다.

```bash
python3 src/il.py gradcam \
  --checkpoint outputs/act_modular/can_color_sort_act_task/checkpoints/policy_best.ckpt \
  --episode datasets/can_color_sort/episode_000147.hdf5 \
  --frames 120 200 \
  --target action \
  --chunk-step 89 \
  --action-index 1 \
  --target-sign -1 \
  --output-dir outputs/analysis/gradcam/task_episode147_ee_y_negative
```

Joint action index는 오른팔 관절 `0:7`과 grasp `7`이므로 Joint의 한 관절과 Task의
Cartesian 축을 직접 같은 의미로 비교하면 안 된다. 특정 Task 운동의 진단에는
`--target action`을 사용한다. PNG는 RGB, Grad-CAM,
overlay를 담고 `.npz`는 heatmap, 정규화 전 heatmap 최댓값, gradient 절댓값 평균,
예측 chunk, target score를 보존한다. 정규화된 색만 비교하지 말고 원 신호 크기도 함께
확인해야 매우 작은 gradient가 과장되어 보이는 것을 피할 수 있다.

Joint/Task를 공정하게 비교할 때는 두 표현을 world-frame EE 방향으로 통일한다. 아래
폐루프 분석은 Task 출력의 EE Y를 직접 사용하고, Joint 출력은 현재 MuJoCo Jacobian의
Y row와 `action_std`로 투영한다. +Y/-Y를 모두 원본 저장하며 expert trajectory나 성공
결과를 target 선택에 사용하지 않는다.

```bash
PYTHONPATH=src MUJOCO_GL=egl python3 scripts/analyze_closed_loop_ee_y_gradcam.py \
  --output-dir outputs/analysis/closed_loop_gradcam_ee_y_<run>
```

정확한 target 정의와 해석 한계는
[ACT 연구 보고서](research-report.md#8-closed-loop-signed-ee-y-grad-cam)를 참고한다.

Grad-CAM은 상관 기반의 국소 설명이며 색상 사용의 인과적 증거는 아니다. 결론을 낼 때는
성공/실패 episode와 색·상자 배치를 균형 있게 표본화하고, 동일 장면의 색상 가림 또는
교체 실험과 함께 확인한다. 마지막 image feature map이 낮은 공간 해상도이므로 heatmap이
물체 경계보다 넓게 나타나는 것도 정상이다.

## Run the Interactive Policy UI

```bash
python3 src/teleop_app.py
```

`Control Center > ACT Policy` 탭이 `outputs/act/<run>/checkpoints/*.ckpt`를 자동으로
검색한다. training run과 checkpoint를 선택하고 `Max steps`를 정한 다음
`Load + Run ACT Policy`를 누르면 새 창을 만들지 않고 현재 teleop의 MuJoCo model,
data와 렌더 창에서 정책을 실행한다. `Refresh models`는 앱 실행 후 새로 생긴
checkpoint 목록을 다시 읽는다. 캔이 상자 안에서 안정된 것은 상태로만 표시하며,
rollout은 중단하지 않고 home 복귀 동작을 포함해 `Max steps`까지 계속된다.
teleop은 시작할 때부터 학습과 동일한 head 자세, 왼팔 park 자세, 캔 spawn 분포와
target-bin/오른손 collision 설정을 사용하므로 정책을 로드해도 환경이나 자세가
바뀌지 않는다. 정책 모드의 `R`도 로봇을 reset하지 않고 캔과 temporal aggregation만
reset한다. `Max steps`에 도달하거나 `Stop + Return to IK`를 누르면 측정된 현재 손
자세를 IK 목표로 다시 맞춘 뒤 기존 IK 제어로 복귀한다.

checkpoint를 직접 지정해 선택 화면을 건너뛰는 기존 방식도 지원한다.

```bash
python3 src/teleop_app.py \
  --policy-checkpoint outputs/act/<run>/checkpoints/policy_best.ckpt \
  --policy-device auto \
  --policy-seed 1000 \
  --policy-max-steps 500
```

## CLI Layout

`src/il.py`가 가벼운 단일 dispatcher이며 선택된 command 모듈만 지연 import한다.

| 명령 | 구현 모듈 | 책임 |
|---|---|---|
| `record` | `ffw_sh5_grasp.cli.record` | demonstration recorder UI |
| `validate` | `ffw_sh5_grasp.cli.validate` | 전체 HDF5 dataset 사전 검증 |
| `visualize` | `ffw_sh5_grasp.cli.visualize` | recorded RGB를 MP4로 변환 |
| `rerun` | `ffw_sh5_grasp.cli.rerun` | episode Rerun recording/stream |
| `replay` | `ffw_sh5_grasp.cli.replay` | expert action physics replay |
| `train` | `ffw_sh5_grasp.cli.train` | ACT train |
| `train-modular` | `ffw_sh5_grasp.cli.train` | 이전 명령 호환용 `train` 별칭 |
| `evaluate` | `ffw_sh5_grasp.cli.evaluate` | closed-loop evaluation |
| `evaluate-color-sort` | `ffw_sh5_grasp.cli.evaluate_color_sort` | 데이터/표현/PTE 평가 행렬 |
| `compare` | `ffw_sh5_grasp.cli.compare` | expert와 policy action 비교 |
| `gradcam` | `ffw_sh5_grasp.cli.gradcam` | ACT 연속 action target의 카메라별 Grad-CAM |

기존 teleop의 진입점은 `src/teleop_app.py`로 분리되어 있다.

공통 episode 경로 규칙은 `ffw_sh5_grasp.imitation.data.paths`가 소유한다. CLI에서
`--episode`가 있으면 이를 우선하고, 없으면 `--dataset-dir`와 `--episode-idx`로
`episode_000000.hdf5` 형식을 구성한다.
