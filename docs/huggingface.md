# Hugging Face 배포

`v3.0.0`의 색상 분류 데이터와 평가용 ACT 정책은 공개 저장소로 배포되어 있다.

- [FFW-SH5 Can Color Sort 데이터셋](https://huggingface.co/datasets/ggh-png/ffw-sh5-can-color-sort)
- [FFW-SH5 ACT Color Sort 정책](https://huggingface.co/ggh-png/ffw-sh5-act-color-sort)

## 내려받기

```bash
python -m pip install -r requirements-huggingface.txt

hf download ggh-png/ffw-sh5-can-color-sort \
  --repo-type dataset \
  --local-dir datasets/can_color_sort_hf

hf download ggh-png/ffw-sh5-act-color-sort \
  --local-dir outputs/hf/ffw-sh5-act-color-sort
```

데이터 저장소의 `data/episode_*.hdf5` 150개에는 58,676 frame이 들어 있다. 각 episode는
양팔 16D qpos/qvel/action, 양쪽 world-frame EE pose와 `cam_high`, `cam_left_wrist`,
`cam_right_wrist` RGB를 담는다. `dataset_manifest.csv`에서 크기와 SHA-256을 확인한다.

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

## 로컬 파일 검증과 재배포

업로드 전 release 디렉터리를 만들고 schema, 모델 산출물과 hash manifest를 검사한다.

```bash
python3 scripts/prepare_huggingface_release.py

python3 scripts/publish_huggingface.py \
  --dataset-repo-id ggh-png/ffw-sh5-can-color-sort \
  --model-repo-id ggh-png/ffw-sh5-act-color-sort \
  --dry-run
```

실제 배포는 Hugging Face 로그인 후 명시적으로 수행한다. `--public`은 저장소 공개
설정까지 적용하므로 asset과 checkpoint의 배포 권한을 먼저 확인해야 한다.

```bash
hf auth login
HF_XET_HIGH_PERFORMANCE=1 python3 scripts/publish_huggingface.py \
  --dataset-repo-id ggh-png/ffw-sh5-can-color-sort \
  --model-repo-id ggh-png/ffw-sh5-act-color-sort \
  --public
```

현재 카드는 원 프로젝트 asset 전체의 라이선스 검토가 끝나지 않았음을 분명히 하기
위해 `license: other`를 사용한다. 실물 로봇 배포 전에는 카메라 보정, EE 좌표계,
제어 주기, 관절/충돌 안전 계층과 sim-to-real 차이를 다시 검증해야 한다.
