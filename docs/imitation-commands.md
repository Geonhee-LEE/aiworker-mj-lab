# Imitation Learning Command Reference

이 문서는 can-to-box 오른팔 ACT 워크플로의 모든 실행 명령을 한 곳에 정리한다.
모든 명령은 저장소 루트에서 실행한다.

```bash
cd /home/ggh/wsyoon/ffw-sh5-grasp
```

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements-imitation.txt
wandb login
```

W&B를 사용하지 않으면 `config/imitation/act.yaml`의 `wandb.enabled`를 `false`로
설정한다. GUI 앱은 Linux 데스크톱 OpenGL 세션에서 실행한다.

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
| Joint/Task ACT 학습 | `python3 src/il.py train-modular --config config/imitation/act_color_sort_joint.yaml` | 표현별 독립 checkpoint |
| closed-loop 평가 | `python3 src/il.py evaluate --checkpoint outputs/act/<run>/checkpoints/policy_best.ckpt` | `evaluation.json` |
| 색상 분류 평가 행렬 | `python3 src/il.py evaluate-color-sort` | 4 정책 × 5 PTE × 100회 CSV/JSON |
| expert/policy 비교 | `python3 src/il.py compare --checkpoint outputs/act/<run>/checkpoints/policy_best.ckpt --episode datasets/can_to_box/episode_000000.hdf5` | comparison `.rrd` |
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

Joint/Task 표현 비교와 150 episode 설정은 별도의 modular 명령을 사용한다.

```bash
python3 src/il.py train-modular \
  --config config/imitation/act_color_sort_joint.yaml
python3 src/il.py train-modular \
  --config config/imitation/act_color_sort_task.yaml
python3 src/il.py train-modular \
  --config config/imitation/act_color_sort_joint_aug150.yaml
python3 src/il.py train-modular \
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

또는 직접 진입점을 사용한다.

```bash
python3 src/il.py policy \
  --checkpoint outputs/act/<run>/checkpoints/policy_best.ckpt \
  --device auto --seed 1000 --max-steps 500
```

UI의 `Max steps`에서 rollout 길이를 바꾸고 `Run policy` 버튼으로 시작한다. 정책은
성공하거나 최대 step에 도달하면 자동으로 정지한다. 완료 후 버튼을 다시 누르면 환경과
temporal aggregation을 reset한 뒤 새 rollout을 시작한다. `SPACE`는 실행/정지, `N`은
한 action step, `R`은 robot, 고정 head, 고정 left arm, can pose와 temporal
aggregation을 함께 reset한다. 이 UI는 checkpoint에 저장된 카메라 목록과 policy
index를 사용한다.

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
| `train-modular` | `ffw_sh5_grasp.cli.train_modular` | Joint/Task ACT train |
| `evaluate` | `ffw_sh5_grasp.cli.evaluate` | closed-loop evaluation |
| `evaluate-color-sort` | `ffw_sh5_grasp.cli.evaluate_color_sort` | 데이터/표현/PTE 평가 행렬 |
| `compare` | `ffw_sh5_grasp.cli.compare` | expert와 policy action 비교 |
| `policy` | `ffw_sh5_grasp.cli.policy` | 독립 ACT policy UI |

기존 teleop의 진입점은 `src/teleop_app.py`로 분리되어 있다.

공통 episode 경로 규칙은 `ffw_sh5_grasp.imitation.data.paths`가 소유한다. CLI에서
`--episode`가 있으면 이를 우선하고, 없으면 `--dataset-dir`와 `--episode-idx`로
`episode_000000.hdf5` 형식을 구성한다.
