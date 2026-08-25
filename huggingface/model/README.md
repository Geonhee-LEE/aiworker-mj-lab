---
library_name: pytorch
license: other
tags:
- robotics
- imitation-learning
- act
- mujoco
- temporal-ensemble
- pte
datasets:
- {{DATASET_REPO_ID}}
---

# FFW-SH5 ACT Can Color Sort Policies

FFW-SH5 MuJoCo 색상 분류 작업에 대해 같은 ACT 구조로 학습한 네 정책입니다.
출력 표현 공간과 학습 데이터 조건만 교체하여 비교했습니다.

이 카드는 코드 릴리즈 `v3.1.0`과 함께 검증되었습니다.

## Included policies

| 경로 | Episodes | Representation | State / action |
|---|---:|---|---|
| `policies/d097_joint` | 97 | Joint | 오른팔 관절 7 + grasp |
| `policies/d097_task` | 97 | Task | 오른팔 EE pose 7 + grasp |
| `policies/d150_joint` | 150 | Joint | 오른팔 관절 7 + grasp |
| `policies/d150_task` | 150 | Task | 오른팔 EE pose 7 + grasp |

각 경로에는 아래 파일만 포함합니다.

```text
checkpoints/policy_best.ckpt
config.yaml
dataset_stats.pkl
episode_splits.json
metrics/metrics.csv
plots/*.png
```

optimizer state가 포함되어 약 3배 큰 `policy_last.ckpt`와 Rerun/W&B 로그는
배포에서 제외했습니다.

## Architecture

- ACT: CVAE Transformer
- Visual backbone: ResNet-18
- Cameras: `cam_high`, `cam_right_wrist`
- State/action dimension: 8
- Action chunk: 90 steps
- Control rate: 25 Hz
- Training epochs: 2,000

Task-space 정책은 `[x, y, z, qw, qx, qy, qz, grasp]`를 예측하며 실행 시
프로젝트의 IK solver로 joint target으로 변환합니다.

## Evaluation

각 조건을 100회씩, 총 2,000 rollouts로 평가했습니다. 실패에는 20초 timeout을
부여한 penalized completion time을 사용했습니다.

| Policy | f=0 success | f=5 success | f=10 success | f=10 speedup |
|---|---:|---:|---:|---:|
| D97 Joint | 79% | 80% | 75% | 1.174× |
| D97 Task | 95% | 100% | 97% | 1.507× |
| D150 Joint | 100% | 100% | 98% | 1.424× |
| D150 Task | 83% | 100% | 90% | 1.460× |

전체 결과와 95% confidence interval은 `evaluation/experiment_summary.csv`에
포함되어 있습니다. PTE `f=5`는 가장 안정적인 공통 운용점이었고, `f>=15`에서는
성공률이 급격히 감소했습니다.

## Inference

먼저 모델을 고정 revision으로 내려받습니다.

```bash
hf download ggh-png/ffw-sh5-act-color-sort \
  --revision v3.1.0 --local-dir outputs/hf/ffw-sh5-act-color-sort
```

```bash
python3 src/teleop_app.py --env 1 \
  --policy-checkpoint outputs/hf/ffw-sh5-act-color-sort/policies/d150_joint/checkpoints/policy_best.ckpt \
  --policy-representation auto \
  --policy-pte-steps 5
```

실행 코드와 MuJoCo 환경은 {{CODE_REPO_URL}}에서 확인할 수 있습니다.

## Limitations

- MuJoCo 색상 분류 환경에서만 정량 검증했습니다.
- 실제 로봇 배포 성능은 측정하지 않았습니다.
- Task-space 정책 성능은 IK gain, collision constraint와 좌표계에 의존합니다.
- 큰 PTE look-ahead는 접촉·파지 단계를 건너뛰어 실패할 수 있습니다.

## License

코드, checkpoint와 학습에 사용된 asset의 재배포 조건을 최종 확인하기 전까지
`license: other`로 표시합니다.
