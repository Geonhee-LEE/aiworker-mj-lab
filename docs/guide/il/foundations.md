# 행동 복제와 데이터

## 모방학습의 문제 정의

모방학습은 전문가 정책이 만든 demonstration으로부터 정책을 학습한다. 이 프로젝트의
기본 방법은 행동 복제(Behavior Cloning, BC)다. 전문가 데이터셋을

\[
\mathcal D = \{(o_t, a_t)\}
\]

라고 하면, 정책 \(\pi_\theta\)가 관측 \(o_t\)에서 전문가 행동 \(a_t\)를 예측하도록
지도학습한다.

\[
\theta^* = \arg\min_\theta
\sum_{(o_t,a_t)\in\mathcal D}
\ell(\pi_\theta(o_t), a_t)
\]

여기서 관측은 카메라 영상과 로봇 상태이고, 행동은 환경에 전달할 관절 target이다.
ACT도 이 행동 복제 문제를 풀지만 한 시점의 행동 하나가 아니라 **미래 행동 묶음**을
한 번에 예측한다.

## Observation, action, state

- **State**: 환경을 완전히 기술하는 값이다. 시뮬레이터 내부 물체 pose까지 포함할 수
  있지만 실제 로봇에서는 모두 알기 어렵다.
- **Observation**: 정책이 실제로 볼 수 있는 정보다. 현재 구현에서는 qpos와 RGB다.
- **Action**: 정책이 환경에 지시하는 값이다. 현재 구현에서는 오른팔 관절과 grasp의
  절대 target이다.
- **Policy**: 관측으로부터 행동 또는 행동의 분포를 만드는 함수다.

카메라만으로 가려짐과 깊이를 완전히 해소할 수 없고 qpos만으로 물체 위치를 알 수
없다. 두 입력은 서로 대체 관계가 아니라 보완 관계다.

## 현재 데이터 계약

episode의 timestep `t`에서 학습 sample은 다음처럼 만들어진다.

```text
입력
  observations/qpos[t, 8:16]
  images/cam_high[t]
  images/cam_right_wrist[t]

정답
  action[t:t+K, 8:16]
  is_pad[0:K]
```

`K`는 `chunk_size`다. episode 끝을 지나 부족한 action은 padding하고, loss를 계산할
때 `is_pad` mask로 제외한다. 영상과 qpos와 action의 timestep이 어긋나면 모델은
인과관계 대신 지연 오차를 학습하므로, 기록 주기와 index 정렬은 모델 구조만큼 중요하다.

| 시점 | 정책 입력 | 정답/출력 |
|---|---|---|
| 학습 | 정규화 qpos, 정규화 RGB, 정답 action chunk | 예측 chunk, posterior `mu/logvar`, loss |
| 검증 | 학습과 동일 | 학습에 사용하지 않은 episode의 loss |
| 추론 | 정규화 qpos, 정규화 RGB | 역정규화할 예측 action chunk |

정규화 통계는 train split에서만 계산하고 checkpoint와 같은 run 디렉터리에 보관한다.
추론 시 다른 데이터의 통계를 쓰면 입력과 출력의 물리 단위가 달라지므로, 가중치가
정상이어도 로봇 행동이 잘못될 수 있다.

## 행동 복제의 핵심 한계

### Covariate shift

학습 데이터는 전문가가 방문한 상태로 구성된다. 추론 중 정책이 작은 오차를 내면
전문가 데이터에 없던 상태로 이동하고, 그 상태에서 더 큰 오차를 낼 수 있다. 이를
covariate shift와 compounding error라고 한다.

따라서 낮은 validation loss만으로 작업 성공을 판단할 수 없다. 다음을 함께 본다.

- 시작 물체 pose를 바꾼 closed-loop rollout 성공률
- grasp, 운반, 투입, 복귀 같은 작업 단계별 실패 위치
- 관절 속도와 target jump
- 카메라 가림, 조명, 배경 변화에 대한 민감도

### Multi-modality

같은 관측에서도 물체 왼쪽이나 오른쪽으로 접근하는 여러 올바른 행동이 존재할 수
있다. 단순 L2 회귀는 이 행동들을 평균내어 어느 demonstration에도 없는 행동을 만들기
쉽다. ACT는 CVAE latent로 demonstration의 행동 style을 표현하고, L1 loss와 action
chunking으로 일관된 trajectory 구간을 학습한다.

### Open-loop metric과 closed-loop 성능

L1 loss는 기록된 target을 얼마나 가깝게 예측했는지 나타낼 뿐, 접촉 이후의 물리
상호작용이나 task 완료를 직접 측정하지 않는다. checkpoint 선택에는 validation loss를
사용할 수 있지만 최종 판단은 환경에서 rollout으로 해야 한다.

## 데이터 수집 체크리스트

- qpos, 모든 camera frame, action이 같은 control tick에 대응하는가?
- 성공 episode만 쓸지, 실패도 별도로 보존할지 기준이 일관적인가?
- 시작 pose와 접근 경로가 지나치게 한 가지로 고정되지 않았는가?
- 캔 투입 후 원점 복귀까지 demonstration에 실제로 포함되어 있는가?
- episode 종료 전에 목표 행동이 잘리지 않았는가?
- 카메라 이름, 해상도, action 차원이 설정과 일치하는가?

다음으로 [RNN과 Transformer](sequence-models.md)에서 시간 문맥을 모델링하는 방법을
비교한다.
