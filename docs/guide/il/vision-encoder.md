# CNN과 ResNet18

ACT의 Transformer는 RGB 원본을 직접 처리하지 않는다. 먼저 CNN 기반의 ResNet18이
영상에서 물체, 로봇 손과 공간 배치를 나타내는 feature map을 만든다. 따라서 ResNet은
선택적인 부가 기능이 아니라 현재 ACT 정책의 시각 encoder다.

## CNN이 하는 일

convolution filter는 이미지의 작은 영역을 훑으며 모서리, 색 변화, 질감 같은 지역
특징을 찾는다. 여러 convolution layer를 통과할수록 각 feature가 보는 범위인 receptive
field가 넓어지고, 더 복합적인 형태를 표현할 수 있다.

입력이 `[3, H, W]` RGB라면 CNN 출력은 보통 `[C, H', W']` feature map이다.

- `C`: 학습된 시각 특징의 channel 수
- `H', W'`: 원본보다 작아진 공간 격자
- 격자의 각 위치: 해당 이미지 영역에서 검출된 특징 vector

## ResNet의 residual connection

네트워크가 깊어지면 단순히 layer를 더 쌓는 것만으로 학습이 좋아지지 않을 수 있다.
ResNet block은 학습할 변환 `F(x)`에 입력 `x`를 바로 더한다.

\[
y = F(x) + x
\]

이 skip connection은 gradient가 깊은 layer까지 전달되는 경로를 제공하고, 필요하면
block이 입력에 작은 수정만 학습하게 한다. ResNet18은 비교적 가벼우면서 검증된
backbone이라 ACT 공개 구현에서도 사용된다.

## ACT가 feature map을 유지하는 이유

일반 이미지 분류기는 마지막에 global average pooling으로 공간 축을 하나의 vector로
줄인다. 그러나 로봇 정책은 캔과 상자의 종류뿐 아니라 화면에서의 위치도 알아야 한다.
현재 구현은 ResNet18의 stride-32 출력 feature map을 유지하고 1×1 convolution으로
Transformer의 `hidden_dim`에 맞춘다.

```text
RGB [B,N,3,H,W]
  → 공유 ResNet18
  → feature [B,N,512,H',W']
  → 1×1 projection
  → token [B,N,H',W',hidden_dim]
  → 2D positional embedding과 함께 Transformer로 전달
```

feature만 펼치면 Transformer는 각 token이 어느 행과 열에서 왔는지 알 수 없다. 그래서
고정 2D sine/cosine positional embedding을 더해 공간 위치를 보존한다.

## 여러 카메라와 가중치 공유

`cam_high`와 `cam_right_wrist`는 하나의 ResNet18을 공유한다. 이는 카메라마다 별도
backbone을 두는 것보다 parameter 수를 줄이고, 공통된 시각 특징을 학습하게 한다.
각 카메라의 feature token은 모두 observation sequence에 포함되므로 Transformer가
전역 시점과 손목 시점에서 필요한 정보를 선택할 수 있다.

## Pretrained weight와 BatchNorm

현재 설정의 `pretrained_backbone: true`는 ImageNet pretrained ResNet18로 시작한다는
뜻이다. 작은 로봇 demonstration 데이터만으로 저수준 시각 특징을 처음부터 학습하는
부담을 줄일 수 있다. 입력 RGB도 같은 ImageNet mean/std로 정규화한다.

현재 backbone은 BatchNorm 통계를 고정해 작은 batch에서 running statistics가 불안정하게
변하는 것을 막는다. pretrained weight를 쓴다는 것과 backbone 전체를 학습하지 않는다는
것은 같은 뜻이 아니다. optimizer의 backbone learning rate 설정에 따라 convolution
weight는 계속 fine-tuning될 수 있다.

## 확인해야 할 실수

- RGB channel 순서가 HDF5 기록과 loader에서 일치하는가?
- 입력 값이 `[0,1]` 범위로 변환된 뒤 ImageNet 정규화를 거치는가?
- 모든 카메라의 해상도가 같고 checkpoint의 camera 순서와 일치하는가?
- feature를 global pooling해 공간 위치를 없애지 않았는가?
- 2D positional embedding의 shape이 feature map과 일치하는가?
- 훈련과 추론이 같은 crop, resize와 normalization을 사용하는가?

구현은 `src/ffw_sh5_grasp/imitation/act/backbone.py`와
`ACTPolicy._encode_observation()`에서 확인할 수 있다. 다음으로
[RNN과 Transformer](sequence-models.md)에서 이 시각 token과 action query를 연결하는
시계열 모델을 다룬다.
