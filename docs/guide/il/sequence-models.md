# RNN과 Transformer

로봇 행동은 순서가 중요하다. 캔을 잡기 전에 손을 닫거나, 투입 직후 복귀 동작을
건너뛰면 각 관절 target 자체가 그럴듯해도 작업은 실패한다. 시계열 모델은 현재 관측을
시간 문맥과 연결해 이러한 순서를 표현한다.

## RNN 계열

RNN(Recurrent Neural Network)은 이전 hidden state와 현재 입력으로 새 hidden state를
만든다.

\[
h_t = f_\theta(x_t, h_{t-1}), \qquad
y_t = g_\theta(h_t)
\]

LSTM과 GRU는 gate를 사용해 오래된 정보를 보존하고 gradient 소실을 줄인 RNN 변형이다.

### 장점

- timestep 단위 streaming 구현이 자연스럽다.
- hidden state 하나로 과거를 요약하므로 짧은 sequence에서는 가볍다.
- 입력 길이가 바뀌어도 같은 recurrent cell을 사용할 수 있다.

### 한계

- timestep을 순서대로 처리하므로 학습 병렬화가 어렵다.
- 긴 sequence의 정보를 하나의 hidden state에 유지하기 어렵다.
- hidden state 오류가 시간이 지나며 누적될 수 있다.

ACT는 RNN이나 LSTM을 사용하지 않는다. RNN을 알아야 하는 이유는 ACT가 시간 문제를
recurrent state 대신 Transformer attention과 action chunk로 푸는 방식을 비교하기
위해서다.

## Transformer

Transformer는 sequence의 token들이 attention으로 서로 필요한 정보를 직접 참조한다.
scaled dot-product attention은 다음과 같다.

\[
\operatorname{Attention}(Q,K,V)=
\operatorname{softmax}\left(\frac{QK^T}{\sqrt{d_k}}\right)V
\]

- Query는 현재 token이 찾는 정보다.
- Key는 각 token이 어떤 정보를 가진지 나타낸다.
- Value는 attention 가중합으로 실제 전달되는 내용이다.
- Multi-head attention은 서로 다른 관계를 여러 head가 병렬로 학습하게 한다.

attention만으로는 token 순서를 알 수 없으므로 positional embedding을 더한다. action
sequence에는 1D 순서 정보가 필요하고, 카메라 feature map에는 이미지의 행과 열을
표현하는 2D 위치 정보가 필요하다.

## Encoder와 decoder

- **Encoder self-attention**은 입력 token 사이의 관계를 표현한다.
- **Decoder self-attention**은 output query 사이의 관계를 표현한다.
- **Cross-attention**은 output query가 encoder의 관측 feature를 참조하게 한다.

ACT에는 서로 목적이 다른 Transformer 경로가 있다.

1. CVAE posterior encoder가 `[CLS]`, qpos, 정답 action chunk를 읽어 행동 style latent를
   만든다.
2. observation encoder가 이미지 feature, qpos, latent를 결합한다.
3. action query `K`개가 decoder를 지나 미래 `K`개 행동을 동시에 출력한다.

## CNN과 ResNet18의 역할

Transformer에 원본 RGB 픽셀을 그대로 넣지 않는다. 공유 ResNet18 backbone이 각
카메라 영상을 더 작은 spatial feature map으로 바꾼다. ResNet의 residual block은

\[
y = F(x) + x
\]

처럼 입력을 skip connection으로 더해 깊은 CNN의 학습을 안정화한다. 현재 구현은
global average pooling으로 위치를 없애지 않고, stride-32 feature map과 2D sine/cosine
positional embedding을 유지한다. 이는 캔과 상자의 **무엇**뿐 아니라 **어디**를
구분하기 위해 필요하다.

여러 카메라는 같은 ResNet18 가중치를 공유한다. 각 카메라 feature는 positional
embedding과 함께 펼쳐져 Transformer가 참조하는 observation memory가 된다.
구조와 전처리의 상세 내용은 [CNN과 ResNet18](vision-encoder.md)을 참고한다.

## 모델 비교

| 항목 | RNN/LSTM/GRU | Transformer | ACT에서의 선택 |
|---|---|---|---|
| 과거 정보 | hidden state에 순차 축약 | attention으로 token 직접 참조 | Transformer |
| 학습 병렬화 | timestep 의존성이 큼 | sequence 병렬 처리 가능 | 유리 |
| 긴 의존성 | 유지가 어려울 수 있음 | 직접 attention 가능 | 유리 |
| 실행 상태 | recurrent state 관리 필요 | 관측마다 query 가능 | action chunk 재예측 |
| 출력 | 보통 한 step씩 | 여러 output query 가능 | `K`개 행동 동시 출력 |

ACT의 시간적 일관성은 Transformer 하나에서만 생기지 않는다. 미래 action chunk 전체를
공동 예측하고, 실행 중 매 timestep 얻은 여러 chunk를 temporal ensemble로 합치는 두
장치가 함께 작동한다.

## 현재 코드에서 볼 위치

| 개념 | 모듈 |
|---|---|
| ResNet18과 2D 위치 embedding | `ffw_sh5_grasp.imitation.act.backbone` |
| encoder/decoder attention block | `ffw_sh5_grasp.imitation.act.transformer` |
| token 구성과 action query | `ffw_sh5_grasp.imitation.act.policy` |
| chunk 누적과 temporal ensemble | `ffw_sh5_grasp.imitation.runtime.runner` |

다음으로 [VAE와 CVAE](cvae.md)에서 ACT가 행동의 여러 가능한 style을 어떻게 표현하는지
살펴본다.
