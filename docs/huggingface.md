# 공개 정책·데이터셋 다운로드

`v3.1.0`의 색상 분류 데이터와 평가용 ACT 정책은 공개 저장소로 배포되어 있다. 두
저장소의 `v3.1.0` revision은 코드 릴리즈와 같은 manifest·model card를 가리킨다.

- [FFW-SH5 Can Color Sort 데이터셋](https://huggingface.co/datasets/ggh-png/ffw-sh5-can-color-sort)
- [FFW-SH5 ACT Color Sort 정책](https://huggingface.co/ggh-png/ffw-sh5-act-color-sort)

Git 저장소에는 코드, 설정과 작은 평가 요약만 둔다. 약 321 MB인 checkpoint 여러 개와
150개 HDF5 episode는 Git history에 복제하지 않고 Hugging Face를 단일 공개 원본으로
사용한다. 아래 명령은 움직이는 `main` 대신 코드 릴리즈와 맞는 `v3.1.0` revision을
고정한다.

## 1. 다운로드 도구 설치

[빠른 시작](getting-started.md)에 따라 Python 3.12 가상환경을 활성화한 뒤 설치한다.

```bash
python -m pip install -r requirements-huggingface.txt
hf --help
```

`requirements-huggingface.txt`는 일반 runtime, ACT 학습·평가와 Hugging Face CLI를
모두 포함한다. 공개 저장소를 내려받는 데 로그인은 필요하지 않다.

## 2. 학습된 D97/D150 정책 내려받기

네 정책과 각 checkpoint에 대응하는 normalization statistics, YAML, episode split,
학습 metric을 한 번에 받는다. 중단된 다운로드는 같은 명령으로 다시 시작할 수 있다.

```bash
hf download ggh-png/ffw-sh5-act-color-sort \
  --revision v3.1.0 \
  --local-dir outputs/hf/ffw-sh5-act-color-sort
```

필수 파일이 함께 내려왔는지 확인한다. `dataset_stats.pkl` 없이 checkpoint만 복사하면
정규화가 달라지므로 실행하지 않는다.

!!! warning "직렬화 파일의 출처"
    Checkpoint는 안전한 tensor 전용 모드로 읽지만 `dataset_stats.pkl`은 Python pickle
    형식이다. 이 문서의 공식 저장소 또는 직접 생성해 신뢰할 수 있는 파일만 사용하고,
    제3자가 수정한 checkpoint·통계 파일은 로드하지 않는다.

```bash
test -f outputs/hf/ffw-sh5-act-color-sort/policies/d150_joint/checkpoints/policy_best.ckpt
test -f outputs/hf/ffw-sh5-act-color-sort/policies/d150_joint/dataset_stats.pkl
test -f outputs/hf/ffw-sh5-act-color-sort/policies/d150_task/checkpoints/policy_best.ckpt
echo "policy files OK"
```

다운로드 경로는 다음과 같다.

```text
outputs/hf/ffw-sh5-act-color-sort/
├── policies/
│   ├── d097_joint/
│   ├── d097_task/
│   ├── d150_joint/
│   └── d150_task/
├── evaluation/experiment_summary.csv
└── model_manifest.json
```

### Headless closed-loop 평가

NVIDIA EGL 환경에서 D150 Joint 정책을 PTE `f=5`로 10회 평가하는 예시다. CPU/OSMesa
환경에서는 `MUJOCO_GL=egl`을 `MUJOCO_GL=osmesa`로 바꾼다.

```bash
MUJOCO_GL=egl python src/il.py evaluate \
  --checkpoint outputs/hf/ffw-sh5-act-color-sort/policies/d150_joint/checkpoints/policy_best.ckpt \
  --task can_color_sort \
  --representation auto \
  --pte-steps 5 \
  --num-episodes 10 \
  --max-steps 500 \
  --seed 1000 \
  --no-rerun
```

Task 정책은 checkpoint 경로의 `d150_joint`만 `d150_task`로 바꾸면 된다. `auto`가
checkpoint metadata에서 Joint/Task 표현을 판별한다.

### GUI에서 정책 실행

```bash
python src/teleop_app.py --env 1 \
  --policy-checkpoint outputs/hf/ffw-sh5-act-color-sort/policies/d150_joint/checkpoints/policy_best.ckpt \
  --policy-representation auto \
  --policy-pte-steps 5 \
  --policy-max-steps 500
```

`Control Center → ACT Policy`에서 실행 상태를 확인하고 중지할 수 있다. 다른 공개
정책을 쓰려면 `d097_joint`, `d097_task`, `d150_joint`, `d150_task` 중 하나로 경로만
교체한다.

### D150 Joint/Task와 PTE 동작 비교

<figure markdown>
  ![D150 Joint와 Task 정책에서 PTE f=5, 10, 15, 20을 같은 초기 상태로 비교한 종합 GIF](assets/evaluation/d150-joint-task-pte-f05-f20.gif)
  <figcaption>
    같은 seed 195958와 초록 캔을 사용한 단일 closed-loop rollout. 위 행은 Joint,
    아래 행은 Task이며 열은 왼쪽부터 f=5·10·15·20이다. 외부 관찰 카메라는 로봇,
    테이블, 캔과 두 상자를 함께 보여주기 위한 시각화 전용 뷰다.
  </figcaption>
</figure>

이 GIF는 PTE가 커질 때 접근·파지 전환이 어떻게 달라지는지 보여주는 정성 예시이며,
성공률 자체를 나타내지 않는다. 아래 heatmap은 정책·PTE 조합마다 서로 같은 seed
100개를 사용한 총 2,000회 평가 결과다.

![D97·D150 Joint/Task 정책의 PTE별 100회 성공률 heatmap](assets/evaluation/success-rate-heatmap.svg)

네 정책의 공통 100% 성공 운용점은 `f=5`였다. 자세한 신뢰구간, 완료 시간과 색상별
결과는 [캔 색상 분류 평가 결과](evaluation-results.md)를 참고한다.

## 3. 학습 데이터셋 내려받기

전체 150-episode HDF5와 manifest를 내려받는다.

```bash
hf download ggh-png/ffw-sh5-can-color-sort \
  --repo-type dataset \
  --revision v3.1.0 \
  --local-dir datasets/can_color_sort_hf
```

실제 episode는 저장소 내부의 `data/` 아래에 있으므로 validation과 학습에서 이 경로를
사용해야 한다.

```bash
python src/il.py validate \
  --dataset-dir datasets/can_color_sort_hf/data \
  --camera cam_high \
  --camera cam_right_wrist

python src/il.py visualize \
  --episode datasets/can_color_sort_hf/data/episode_000000.hdf5 \
  --output outputs/hf/episode_000000.mp4
```

검증은 episode schema, timestep 정렬, camera, dtype, shape와 finite 값을 확인한다.
`dataset_manifest.csv`에는 각 파일 크기와 SHA-256이, `dataset_summary.json`에는 전체
episode·frame·색상 분포가 들어 있다.

데이터 저장소의 `data/episode_*.hdf5` 150개에는 58,676 frame이 들어 있다. 각 episode는
양팔 16D qpos/qvel/action, 양쪽 world-frame EE pose와 `cam_high`, `cam_left_wrist`,
`cam_right_wrist` RGB를 담는다.

![D97과 D150 전체 dataset 및 train split의 색상별 episode 분포](assets/evaluation/dataset-color-composition.svg)

D97은 초록·빨강 97개만 포함하고 D150은 여기에 주황 24개와 파랑 29개를 추가한다.
따라서 D97/D150 비교는 데이터 수뿐 아니라 색상 coverage도 함께 달라지는 비교다.

## 4. 내려받은 데이터로 다시 학습

배포 dataset은 `datasets/can_color_sort_hf/data`에 있지만 기본 학습 YAML은 로컬 수집
경로인 `datasets/can_color_sort`을 가리킨다. 원본 설정을 덮어쓰지 말고 복사본의
`dataset_dir`과 `run_name`을 변경한다.

```bash
cp config/imitation/act_color_sort_joint_aug150.yaml \
  config/imitation/act_color_sort_joint_hf.yaml
```

복사한 YAML에서 다음 두 항목을 바꾼다.

```yaml
run_name: can_color_sort_act_joint_hf
dataset_dir: datasets/can_color_sort_hf/data
```

W&B를 사용하지 않으면 같은 YAML의 `wandb.enabled`를 `false`로 바꾼 뒤 학습한다.

```bash
python src/il.py train \
  --config config/imitation/act_color_sort_joint_hf.yaml
```

Task 표현은 `act_color_sort_task_aug150.yaml`을 복사해 같은 방식으로 연결한다. D97
조건을 재현할 때는 `episode_count: 97`, D150은 `episode_count: 150`을 유지한다.

## 5. 배포 내용

모델 저장소는 아래 네 평가용 best checkpoint를 제공한다.

```text
policies/
├── d097_joint/
├── d097_task/
├── d150_joint/
└── d150_task/
```

각 디렉터리에는 `policy_best.ckpt`, 학습 설정, normalization stats, episode split,
metric과 plot이 있다. optimizer state가 큰 `policy_last.ckpt`, W&B와 Rerun 로그는
재현에 불필요하므로 배포하지 않는다. 전체 2,000-rollout 결과는
`evaluation/experiment_summary.csv`에 포함된다.

## 6. 사용 범위

현재 카드는 원 프로젝트 asset 전체의 라이선스 검토가 끝나지 않았음을 분명히 하기
위해 `license: other`를 사용한다. 실물 로봇 배포 전에는 카메라 보정, EE 좌표계,
제어 주기, 관절/충돌 안전 계층과 sim-to-real 차이를 다시 검증해야 한다.

새 revision을 업로드하는 저장소 유지관리자는
[Hugging Face 재배포](guide/huggingface-publishing.md)를 따른다.
