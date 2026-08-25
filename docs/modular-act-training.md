# Joint/Task ACT training

하나의 `trainer.py`가 YAML의 `representation`에 따라 오른팔 정책의 좌표 표현만
바꿔 학습한다.

| representation | state (8D) | action (8D) |
|---|---|---|
| `joint` | 오른팔 관절 7 + measured grasp 1 | 오른팔 관절 target 7 + grasp command 1 |
| `task` | 오른팔 EE `xyz + quaternion(wxyz)` 7 + measured grasp 1 | joint action을 FK한 EE target 7 + grasp command 1 |

두 설정은 `run_name`과 `representation` 외의 카메라, split seed, training seed,
ACT 구조 및 optimizer가 같다. 원본 HDF5는 수정하지 않으며 task action pose는 학습
시작 때 메모리에 한 번 계산해 캐시한다. Episode의 `model_hash`, EE frame 및
quaternion 순서가 현재 모델과 맞지 않으면 학습을 거부한다.

데이터 취득을 끝낸 후 두 실험을 각각 실행한다. 한 GPU에서 동시에 실행하면 비교
시간과 처리량이 왜곡되므로 순차 실행한다.

```bash
python3 src/il.py train \
  --config config/imitation/act_color_sort_joint.yaml

python3 src/il.py train \
  --config config/imitation/act_color_sort_task.yaml
```

150 episode 조건은 기존 run을 덮어쓰지 않는 별도 설정을 사용한다.

```bash
python3 src/il.py train \
  --config config/imitation/act_color_sort_joint_aug150.yaml
python3 src/il.py train \
  --config config/imitation/act_color_sort_task_aug150.yaml
```

출력은 기존 run과 분리된다.

```text
outputs/act_modular/
├── can_color_sort_act_joint/
└── can_color_sort_act_task/
```

Validation은 매 epoch 실행한다. 약 1 GB인 resume checkpoint와 plot은 공식 ACT
주기에 맞춰 100 epoch마다, 마지막 epoch에는 무조건 저장한다. Best checkpoint는
optimizer를 제외한 평가용 파일이고, last checkpoint만 optimizer를 포함한다.

Joint/task checkpoint 모두 `teleop_app.py`에서 실행할 수 있다. 기본 `auto` 모드는
checkpoint의 `representation` metadata를 읽는다. 명시적으로 검사하려면
`--policy-representation joint` 또는 `task`를 추가한다.

```bash
python3 src/teleop_app.py --env 1 \
  --policy-checkpoint outputs/act_modular/can_color_sort_act_joint/checkpoints/policy_best.ckpt \
  --policy-representation auto

python3 src/teleop_app.py --env 1 \
  --policy-checkpoint outputs/act_modular/can_color_sort_act_task/checkpoints/policy_best.ckpt \
  --policy-representation auto \
  --policy-ik-speed-scale 3.0 \
  --policy-pte-steps 0 \
  --policy-rerun
```

Task runner는 현재 오른손 world-frame EE pose와 grasp를 8D 입력으로 구성한다. 출력
chunk의 quaternion은 부호를 정렬하고 정규화한 뒤 temporal ensemble하며, 실행 pose는
기존 bounded differential IK에 `active_sides=("r",)`, `whole_body_enabled=False`로
전달한다. 따라서 base/lift/왼팔은 고정되고 오른팔 관절 한계와 collision constraint는
계속 적용된다. Joint runner는 기존처럼 8D 오른팔 출력을 16D 환경 action으로 복원한다.
Task policy의 pose 추종 gain은 기본 3배이며 `--policy-ik-speed-scale`로 바꿀 수 있다.
이 배율은 정책 실행 중에만 적용되고 관절 속도·관절 범위·충돌 CBF 상한은 바꾸지 않는다.

Teleop ACT 패널의 `Policy representation`은 `AUTO`/`JOINT`/`TASK` metadata로 run
목록을 필터링한다. `PTE future steps`는 `0`일 때 기존 ACT와 같고, 양수일 때 현재
시점에 `t+f` action 후보를 ensemble한다. 실행 중 값을 바꾸면 이전 후보 buffer를
비우고 다음 frame에 새 chunk를 추론한다. CLI에서는 `--policy-pte-steps`로 지정한다.

`--policy-rerun`은 9877 포트로 Viewer를 열고 checkpoint run의 `rerun/` 아래에
`teleop_<representation>_<timestamp>.rrd`를 동시에 저장한다. Task rollout에는 카메라,
현재 EE pose, 8D task target, predicted chunk, PTE offset·target timestep·후보 수,
IK 오차·충돌 지표와 IK 후 실제 16D joint action이 분리되어 기록된다. 포트는
`--policy-rerun-port`로 변경할 수 있다. 제어는 25 Hz를 유지하고 Rerun 이미지는 JPEG
85 품질, 기본 5 Hz로 기록한다. `--policy-rerun-hz`로 기록 주기만 변경할 수 있다.

## 고정 평가 행렬

네 정책에 PTE `f=0,5,10,15,20`을 적용하고 조건마다 같은 seed로 100회 평가한다.
`--dry-run`은 checkpoint를 읽거나 물리를 실행하지 않고 2,000개 rollout 계획만 확인한다.

```bash
python3 src/il.py evaluate-color-sort --dry-run

MUJOCO_GL=egl python3 src/il.py evaluate-color-sort \
  --num-episodes 100 \
  --pte-steps 0 5 10 15 20 \
  --output-dir outputs/evaluation/can_color_sort_pte_m005
```

각 cell은 성공률과 Wilson 95% 신뢰구간, 성공 episode 완료 시간, 실패를 20초로
계산한 penalized time, 추론 시간 및 task IK 안전 지표를 저장한다. 이 프로젝트의
2,000-rollout 결과에서 `f=5`는 네 정책 모두 100% 성공한 공통 운용점이었다. `f=10`은
더 빨랐지만 일부 성공률을 잃었고, `f>=15`는 접촉·파지 구간을 지나치게 앞서가며
성공률이 급감했다.

| Policy | f=0 성공률 | f=5 성공률 | f=10 성공률 | f=10 속도 향상 |
|---|---:|---:|---:|---:|
| D97 Joint | 79% | 80% | 75% | 1.174× |
| D97 Task | 95% | 100% | 97% | 1.507× |
| D150 Joint | 100% | 100% | 98% | 1.424× |
| D150 Task | 83% | 100% | 90% | 1.460× |

여기서 D97은 초기 100개 수집본에서 실패 episode를 제외한 성공 97개다. 따라서
D97/D150 비교는 순수한 표본 수뿐 아니라 후반에 추가한 주황·파랑 색상 비율도 함께
달라지는 실험이라는 한계가 있다. 공개 checkpoint와 전체 CSV는
[Hugging Face 배포](huggingface.md)에서 받을 수 있다.

성공률 신뢰구간, 동일 seed paired 비교, 색상별 ID/OOD 성능, penalized time과 IK
안전 지표까지 포함한 분석은 [캔 색상 분류 평가 결과](evaluation-results.md)에서 본다.
