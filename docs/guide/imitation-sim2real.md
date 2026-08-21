# Arm-only ALOHA/ACT 파이프라인

현재 구현은 ROS와 Whole-body IK를 사용하지 않는 첫 모방학습 경로다. base와 lift는
home reference에, head는 작업대를 보는 고정 pose에 유지된다. can-to-box v1은 오른팔 7축과 오른손 grasp만
조작하고, 왼팔은 손바닥이 월드 +Z를 향하는 주차 자세로 고정한다.

모든 실행 명령과 옵션은 [IL 명령어 레퍼런스](../imitation-commands.md)에 정리되어 있다.
논문 구조와 현재 시스템의 차이는 [ACT 구현과 논문 대응](act-implementation.md)에
정리되어 있다.

```text
GizmoLeader 또는 ACT
  → absolute 16D joint action
  → ArmTorqueController + grasp synergy
  → MuJoCo physics
  → 16D recorded state + RGB 2개 policy observation
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

두 policy 카메라의 위치와 방향은 작업공간을 보도록 임의로 맞춘 값이 아니다.
`cam_high`는 ROBOTIS FFW-SH5의 `head_link2 → zedm_camera_link →
zedm_camera_center → zedm_left_camera_frame` 브라켓 체인을 사용한다.
손목 두 곳은 원본 D405와 같이 `link7 → bottom_screw_frame → camera_link`
체인을 사용하며, 왼쪽과 오른쪽 모두 동일한 브라켓 변환이다.

MuJoCo의 camera 축(+X right, +Y up, -Z forward)은 ROS optical 축(+X
right, +Y down, +Z forward)에 맞게 변환한다. Head camera를 별도로
기울이지 않고 `imitation.head_fixed_position_rad`의 목 pitch를 이용해
작업대를 본다.
조작용 Gizmo 목표, grasp site, 상자 중심 site는 메인 GUI에서만 보이고
HDF5와 Rerun에 저장되는 policy camera 영상에서는 제외된다.

HDF5는 ALOHA 호환성을 위해 16D 양팔 상태/action을 보존한다. 그러나 오른팔 ACT
정책은 index 8..15의 오른팔 qpos/action과 다음 두 RGB만 읽는다. 왼팔 8D 값과
`cam_left_wrist`는 기록 호환성 외에는 학습 및 추론에 전달되지 않는다.

```python
{
    "qpos": float32[16],
    "qvel": float32[16],
    "images": {
        "cam_high": uint8[H,W,3],
        "cam_right_wrist": uint8[H,W,3],
    },
}
```

## Demonstration 기록

```bash
python3 src/il.py record --task-name can_to_box
```

실행하면 조작용 recorder 창과 live Rerun Viewer가 함께 열린다. Rerun은 녹화 전
preview부터 `cam_high`, `cam_right_wrist` policy camera와 16D qpos/qvel/action, task 성공 여부, 녹화 상태와
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
python3 src/il.py validate \
  --dataset-dir datasets/can_to_box \
  --camera cam_high --camera cam_right_wrist
python3 src/il.py visualize \
  --episode datasets/can_to_box/episode_000000.hdf5
python3 src/il.py rerun \
  --episode datasets/can_to_box/episode_000000.hdf5
python3 src/il.py replay \
  --episode datasets/can_to_box/episode_000000.hdf5
```

일반 viewer는 저장된 카메라 영상을 나란히 둔 MP4를 만들고, Rerun viewer는 같은 `frame`
timeline에 RGB, qpos/qvel, expert action을 기록한다. replay는 저장 seed와 action을
같은 actuator 경로에 다시 적용해 policy qpos 오차를 검사한다.

## ACT 학습과 산출물

```bash
python3 src/il.py train --config config/imitation/act.yaml
```

loader는 episode 단위로 train/validation/test를 나누고, 매 epoch 각 episode에서 임의
timestep 하나와 미래 action chunk를 읽는다. RGB 전체를 RAM에 적재하지 않는다. 오른팔
정책은 오른팔 qpos와 `cam_high`, `cam_right_wrist` RGB, padded future action chunk만
사용한다. 먼저 성공 episode 하나만 사용해 overfit할 수 있는지 확인한 뒤 10, 50,
100개 순으로 확장한다.

W&B를 사용하려면 `wandb login`을 한 번 실행하고, `config/imitation/act.yaml`의
`wandb.enabled`, `wandb.project`, 선택적인 `wandb.entity`를 설정한다. 매 epoch의
train/validation loss, L1, KL, learning rate, global step 및 elapsed time이
해당 W&B run에 기록된다. 연결하지 않을 환경에서는 `wandb.enabled: false`로 둔다.

## Interactive Policy UI

학습된 checkpoint는 기존 teleop 진입점의 `ACT Policy` 탭에서 선택한다. 정책 모드는
새 창이나 별도 MuJoCo model을 만들지 않고 현재 teleop의 model, data와 렌더 창에
arm-only 제어기를 연결한다. Whole-body IK나 Gizmo 입력은 정책 실행 중 물리에
적용하지 않으며, checkpoint의 카메라 목록과 policy index를 그대로 사용한다.
따라서 학습과 추론의 observation 계약이 동일하다.

```bash
python3 src/teleop_app.py
```

`Load + Run ACT Policy`는 선택한 checkpoint를 현재 창에 로드하고 바로 실행한다.
`SPACE`는 연속 실행/일시정지, `N`은 한 policy action만 실행, `R`은 로봇·고정
왼팔·고정 head 자세를 유지한 채 캔 pose와 policy temporal state만 reset한다. 캔이
상자 안에 안정되더라도
rollout을 조기 종료하지 않으므로 시연에 포함된 오른팔 home 복귀가 계속 실행된다.
UI는 checkpoint가 사용하는 각 카메라의 frame 유효성, task error, 현재 policy
frame을 표시한다. head는
`imitation.head_fixed_position_rad`에 매 reset과 actuator step마다 고정되며,
`cam_high`는 이 head pose의 ZED-M camera chain을 사용한다.

일반 teleop도 시작부터 같은 head/왼팔/can spawn과 target-bin collision 설정을
사용한다. 따라서 checkpoint를 로드할 때 별도 환경 reset이나 자세 점프가 없다.
rollout이 `Max steps`에 도달하거나 사용자가 `Stop + Return to IK`를 누르면 현재 측정
손 자세를 새 IK 목표로 rebase하고 policy 동안의 base hold를 해제한 뒤 IK 제어로
돌아간다.

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

학습 metric은 epoch 기준 train/validation loss, L1, KL, learning rate,
global step과 elapsed time을 남긴다. PNG loss에는 최소 validation loss와 best epoch를
표시한다.

## Closed-loop 평가

```bash
python3 src/il.py evaluate \
  --checkpoint outputs/act/<run>/checkpoints/policy_best.ckpt

python3 src/il.py compare \
  --checkpoint outputs/act/<run>/checkpoints/policy_best.ckpt \
  --episode datasets/can_to_box/episode_000000.hdf5
```

ACT가 낸 `[K,8]` 오른팔 action chunk는 temporal aggregation을 거쳐 현재 16D action으로
확장된다. 확장된 action의 왼팔 8D 값은 고정 주차 pose와 open grasp 상수다. 각
rollout `.rrd`에는 카메라, qpos, predicted chunk tensor, executed action, success와
object error가 함께 기록된다. `evaluation.json`은 success rate, episode length,
final error, action magnitude와 action delta를 집계한다.

## 실기 전환 경계

simulation의 `ArmTorqueController`는 MuJoCo bias force에 의존하므로 실물에 직접 쓸
수 없다. 실기 adapter는 동일한 16D 측정/명령 계약 아래 vendor driver, limit,
watchdog와 E-stop을 제공해야 한다. 카메라 frame, encoder zero/sign, joint limit,
지연과 명령 rate limit을 shadow mode에서 먼저 검증한다.
