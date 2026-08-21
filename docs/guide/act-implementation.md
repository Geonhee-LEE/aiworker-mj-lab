# ACT 구현과 논문 대응

이 구현은 ALOHA 논문의 ACT 알고리즘을 FFW-SH5 can-to-box 환경에 맞춘 것이다.
모델 구조와 학습 목적은 논문 및 공개 구현을 따르지만, 로봇 자유도·카메라 수·제어
주기는 하드웨어와 현재 task에 맞게 다르다. 따라서 "ALOHA와 같은 ACT"와 "ALOHA
하드웨어를 그대로 복제"하는 것은 구분해야 한다.

ACT를 처음 학습한다면 먼저 [IL 전체 안내](il/index.md)와
[ACT 아키텍처](il/act.md)를 읽는다. 이 문서는 개념 설명보다 논문 설정과 현재 코드의
정확한 대응 관계를 확인하는 레퍼런스다.

- 논문: [Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware](https://arxiv.org/abs/2304.13705)
- 기준 코드: [tonyzhaozh/act](https://github.com/tonyzhaozh/act)

## 일치하는 부분과 시스템 적응

| 항목 | ALOHA 논문 | 이 저장소 |
|---|---|---|
| 이미지 encoder | 공유 ResNet18, 공간 feature map 유지 | 동일 |
| 이미지 위치 정보 | 고정 2D sine/cosine embedding | 동일 |
| style encoder | `[CLS] + qpos + action chunk` transformer | 동일 |
| policy | CVAE transformer encoder/decoder | 동일 |
| 추론 latent | 표준정규 prior의 평균 `z=0` | 동일 |
| loss | masked L1 + `beta * KL`, `beta=10` | 동일 |
| action chunk | 기본 90 step | 동일 |
| temporal ensemble | 매 step query, 같은 target 시점 예측만 가중 평균 | 동일 |
| policy state/action | 양팔 14D | 오른팔 7축 + grasp synergy, 8D |
| 저장 schema | ALOHA HDF5 | left-first 16D 호환 schema |
| 카메라 | RGB 4개, 480×640 | `cam_high`, `cam_right_wrist`, 240×320 |
| 제어 주기 | 50 Hz | 25 Hz |

16D 파일에서 오른쪽 index `8..15`만 정책에 넣는다. 왼팔 값은 기록·replay 계약을
위해 남기고, 환경이 실행 직전에 주차 자세로 강제한다. grasp 하나가 실제 손가락
관절들의 선형 synergy로 확장되므로 ALOHA의 평행 gripper 좌표와 같은 역할을 한다.

!!! note "90 step의 시간 길이"

    논문은 50 Hz에서 90 step, 현재 환경은 25 Hz에서 90 step이다. 따라서 현재 chunk는
    3.6초를 덮는다. 논문의 1.8초 horizon을 시간 기준으로 맞추는 실험은
    `chunk_size: 45`로 별도 run을 만들어 비교한다. 기존 run의 checkpoint와
    `dataset_stats.pkl`을 섞지 않는다.

## 한 sample이 흐르는 경로

```text
episode_*.hdf5
  ├─ observations/qpos[:, 8:16] ─ normalize ─┐
  ├─ images/{high,right_wrist} ─ ResNet18 ───┼─ ACT decoder ─ [K,8]
  └─ action[t:t+K, 8:16] ─ style encoder ─ z ┘       │
                                                     └─ temporal ensemble
                                                           │
                                        16D로 확장 → env.prepare_action
```

학습 dataset은 episode마다 임의 timestep 하나를 고르고 그 시점부터 최대 `K`개의
미래 action을 만든다. episode 끝을 넘는 부분은 padding mask로 제외한다. 카메라
배열은 HDF5에서 해당 frame만 읽으므로 episode 수가 늘어도 전체 RGB를 RAM에 올리지
않는다. normalization 통계는 train split의 qpos와 action만으로 계산하며 표준편차의
최솟값은 공식 loader와 같은 `1e-2`다.

## 모델 내부

1. 각 RGB를 ImageNet 통계로 정규화하고 하나의 pretrained ResNet18을 공유해 처리한다.
2. global pooling 없이 `H'×W'×512` feature map을 유지한다.
3. 2D 위치 embedding, qpos token, latent token을 observation encoder에 넣는다.
4. 학습 시 posterior encoder가 qpos와 정답 action chunk로 `mu`, `logvar`를 만든다.
5. action query `K`개가 decoder cross-attention을 거쳐 절대 관절 target을 출력한다.
6. 추론 시 posterior encoder를 버리고 latent를 항상 0으로 둔다.

설정 기본값 `hidden_dim=512`, encoder 4층, decoder 7층,
`feedforward_dim=3200`, head 8개, dropout 0.1, learning rate `1e-5`는 논문 Table III와
맞춘 값이다. FFW-SH5 차원과 카메라 수는 checkpoint의 `policy_config`에 함께 저장된다.

## Loss와 temporal ensemble

padding을 0으로 mask한 L1 reconstruction과 latent 차원 합 KL을 사용한다.

\[
L = \operatorname{mean}(|\hat a-a|\,M)
  + \beta\operatorname{mean}\left[-\frac12\sum_j
  (1+\log\sigma_j^2-\mu_j^2-\sigma_j^2)\right]
\]

padding 여부를 예측하는 head는 공개 ACT 모델과 checkpoint 구조를 맞추기 위해 있지만
별도 BCE loss로 학습하지 않는다.

추론은 매 timestep 새 chunk를 예측한다. 현재 시점에 도착한 후보를 생성 시점이 오래된
순서로 놓고 `exp(-m*i)`로 평균한다. 기본 `m=0.01`이다. 이는 인접 timestep의 action을
섞는 일반 smoothing과 달리, 모두 같은 실행 timestep을 대상으로 했던 예측만 합친다.

## 코드 책임

| 파일 | 책임 |
|---|---|
| `act/backbone.py` | ResNet18, frozen BatchNorm, 2D 위치 embedding |
| `act/transformer.py` | DETR 방식 positional attention block |
| `act/policy.py` | CVAE posterior, observation encoder, action decoder, loss |
| `act/training_config.py` | YAML 파싱·검증과 policy 차원 선택 |
| `act/dataset_loader.py` | episode split, train 통계, lazy chunk sampling |
| `act/trainer.py` | seed, DataLoader, optimizer, checkpoint lifecycle |
| `act/training_output.py` | CSV/JSONL과 PNG metric 출력 |
| `runtime/runner.py` | checkpoint 복원, 역정규화, temporal ensemble |

전체 데이터·시뮬레이션·앱 계층의 책임과 정식 import 경로는
[모방학습 코드 구조](imitation-code-structure.md)에 정리되어 있다.

`architecture_version=2` 이전 checkpoint는 공간 정보를 버리던 소형 CNN 구조이므로 새
모델에 load하지 않는다. 수집한 HDF5는 호환되며 새 run name으로 재학습하면 된다.

## 학습 전 gate

```bash
python3 src/il.py validate \
  --dataset-dir datasets/can_to_box \
  --camera cam_high \
  --camera cam_right_wrist

python3 src/il.py train --config config/imitation/act.yaml
```

validator가 episode 수, 성공 수, 총 frame과 schema 오류를 보고한다. 그 다음 성공
episode 하나로 overfit해 loss와 실행 경로를 확인하고, 새 run name으로 전체 dataset을
학습한다. 한 run 안에서 서로 다른 split 통계나 구형 checkpoint를 재사용하지 않는다.
