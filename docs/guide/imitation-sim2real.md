# Arm-only ALOHA/ACT 파이프라인

현재 구현은 ROS와 Whole-body IK를 사용하지 않는 첫 모방학습 경로다. base와 lift는
home reference에, head는 작업대를 보는 고정 pose에 유지된다. can-to-box v1은 오른팔 7축과 오른손 grasp만
조작하고, 왼팔은 손바닥이 월드 +Z를 향하는 주차 자세로 고정한다.

```text
GizmoLeader 또는 ACT
  → absolute 16D joint action
  → ArmTorqueController + grasp synergy
  → MuJoCo physics
  → 16D state + RGB 3개
```

## Can-to-box task와 reset

첫 task는 파란 상자 옆에서 시작하는 캔을 오른팔로 고정된 상자 안에
넣는 것이다. 캔의 기준점은 상자 중심에서 로봇 쪽으로 18 cm 떨어져
있고 reset마다 그 점을 중심으로 반지름 5 cm 이내에서 무작위로 배치된다.
상자는 바닥과 네 벽 모두 collision geom이며 캔·손·팔과 실제 contact를
만든다.
`R`을 누르면 기록 중인 미완성 episode를 폐기하고 오른팔·base·lift를 `home`
기준으로, head를 고정된 하향 pose로, 왼팔을 palm-up 주차 자세로 되돌린 뒤
캔의 x/y만 설정 범위에서 다시
추출한다. 상자는 고정되어 있다. 각 reset의 seed와 실제 초기 캔 위치가 HDF5
attribute에 남으므로 replay할 수 있다.

성공은 캔 중심의 상자 내부 x/y 범위, 높이 범위, 선속도 안정화 조건을 모두 만족할
때만 참이다. 범위는 `config/default.yaml`의 `imitation.task`에 있다.

## Policy 입출력

action과 policy qpos/qvel은 호환성을 위해 다음 left-first 16차원을 유지한다.

```text
left arm joint 1..7, left grasp,
right arm joint 1..7, right grasp
```

grasp는 0(open)에서 1(close)이고 기존 finger controller가 실제 손가락 관절로
확장한다. policy가 finger joint 12개를 직접 예측하지 않는다. `env.step(action)`은
robot qpos를 덮어쓰지 않고 팔 PD+bias torque와 손 position actuator를 물리
substep마다 적용한다. 왼쪽 8개 action은 dataset과 실행 시 항상 설정된 palm-up
관절값과 open grasp 상수로 치환된다.

## Policy camera extrinsic

세 카메라의 위치와 방향은 작업공간을 보도록 임의로 맞춘 값이 아니다.
`cam_high`는 ROBOTIS FFW-SH5의 `head_link2 → zedm_camera_link →
zedm_camera_center → zedm_left_camera_frame` 브라켓 체인을 사용한다.
손목 두 곳은 원본 D405와 같이 `link7 → bottom_screw_frame → camera_link`
체인을 사용하며, 왼쪽과 오른쪽 모두 동일한 브라켓 변환이다.

MuJoCo의 camera 축(+X right, +Y up, -Z forward)은 ROS optical 축(+X
right, +Y down, +Z forward)에 맞게 변환한다. Head camera를 별도로
기울이지 않고 `imitation.head_fixed_position_rad`의 목 pitch를 이용해
작업대를 본다.

Observation은 다음 계약을 따른다.

```python
{
    "qpos": float32[16],
    "qvel": float32[16],
    "images": {
        "cam_high": uint8[H,W,3],
        "cam_left_wrist": uint8[H,W,3],
        "cam_right_wrist": uint8[H,W,3],
    },
}
```

## Demonstration 기록

```bash
python3 src/record_episodes.py --task-name can_to_box
```

실행하면 조작용 recorder 창과 live Rerun Viewer가 함께 열린다. Rerun은 녹화 전
preview부터 세 policy camera와 16D qpos/qvel/action, task 성공 여부, 녹화 상태와
episode frame을 같은 실시간 timeline에 표시한다. Rerun을 띄우지 않으려면
`--no-live-rerun`을 사용한다.

| 입력 | 동작 |
|---|---|
| `R` | task pose + random can reset |
| `SPACE` | episode 기록 시작/완료 |
| `BACKSPACE` | 현재 episode 폐기 |
| `Q` | 오른손 잡기/놓기 토글 |
| `E` | 오른팔을 속도 제한 안에서 home 관절 자세로 복귀 |
| `ESC` | 종료 |

GizmoLeader는 오른손 목표를 arm-only differential IK로 7축 target으로 바꾼다.
왼쪽 8개 상수를 포함한 16D action을 follower env가 받으므로 demonstration과 ACT
inference가 같은 실행 경계를 공유한다.

Gizmo IK의 추종 gain과 task/joint 최대 속도는
`config/default.yaml` 아래의 `imitation.teleop`에서 조절한다. 기본값은
선속도 1.0 m/s, 각속도 3.0 rad/s, 관절 속도 4.8 rad/s이다.

매 frame은 step 전에 `obs_t`와 `action_t`를 함께 append한다. 파일 layout은 ALOHA와
호환되는 `/observations/qpos`, `/observations/qvel`,
`/observations/images/<camera>`, `/action`이다. debug에는 EE/object/전체 state를
저장하지만 ACT 입력에는 사용하지 않는다.

## 확인과 replay

```bash
python3 src/visualize_episodes.py \
  --episode datasets/can_to_box/episode_000000.hdf5
python3 src/rerun_episode.py \
  --episode datasets/can_to_box/episode_000000.hdf5
python3 src/replay_episodes.py \
  --episode datasets/can_to_box/episode_000000.hdf5
```

일반 viewer는 세 영상을 나란히 둔 MP4를 만들고, Rerun viewer는 같은 `frame`
timeline에 RGB, qpos/qvel, expert action을 기록한다. replay는 저장 seed와 action을
같은 actuator 경로에 다시 적용해 policy qpos 오차를 검사한다.

## ACT 학습과 산출물

```bash
python3 src/train_act.py --config config/imitation/act.yaml
```

loader는 frame이 아니라 episode 단위로 train/validation을 나눈다. 각 timestep에서
현재 qpos와 세 RGB, padded future action chunk를 만든다. 먼저 성공 episode 하나만
사용해 overfit할 수 있는지 확인한 뒤 10, 50, 100개 순으로 확장한다.

```text
outputs/act/<run>/
├── config.yaml
├── dataset_stats.pkl
├── checkpoints/policy_best.ckpt
├── checkpoints/policy_last.ckpt
├── metrics/metrics.jsonl
├── metrics/metrics.csv
├── plots/{loss,l1,kl,learning_rate}.png
└── rerun/training.rrd
```

학습 metric은 epoch 기준 train/validation loss, L1, KL, padding loss, learning rate,
global step과 elapsed time을 남긴다. PNG loss에는 최소 validation loss와 best epoch를
표시한다.

## Closed-loop 평가

```bash
python3 src/eval_act.py \
  --checkpoint outputs/act/<run>/checkpoints/policy_best.ckpt

python3 src/compare_policy_episode.py \
  --checkpoint outputs/act/<run>/checkpoints/policy_best.ckpt \
  --episode datasets/can_to_box/episode_000000.hdf5
```

ACT가 낸 `[K,16]` chunk는 temporal aggregation을 거쳐 현재 16D action이 된다. 각
rollout `.rrd`에는 카메라, qpos, predicted chunk tensor, executed action, success와
object error가 함께 기록된다. `evaluation.json`은 success rate, episode length,
final error, action magnitude와 action delta를 집계한다.

## 실기 전환 경계

simulation의 `ArmTorqueController`는 MuJoCo bias force에 의존하므로 실물에 직접 쓸
수 없다. 실기 adapter는 동일한 16D 측정/명령 계약 아래 vendor driver, limit,
watchdog와 E-stop을 제공해야 한다. 카메라 frame, encoder zero/sign, joint limit,
지연과 명령 rate limit을 shadow mode에서 먼저 검증한다.
