# Hugging Face 재배포

이 문서는 공개 자산을 **사용하는 절차가 아니라**, 새 release의 dataset과 checkpoint를
검증하고 Hugging Face에 업로드하는 유지관리자 절차다. 정책 또는 데이터를 받으려는
사용자는 [공개 정책·데이터셋](../huggingface.md)을 따른다.

저장소 루트에서 Hugging Face 업로드 도구와 그 하위 runtime/imitation 의존성을 먼저
설치한다.

```bash
python -m pip install -r requirements-huggingface.txt
```

## 1. 로컬 release 검증

업로드 전 release 디렉터리를 만들고 HDF5 schema, 모델 산출물과 hash manifest를
검사한다.

```bash
python scripts/prepare_huggingface_release.py

python scripts/publish_huggingface.py \
  --dataset-repo-id ggh-png/ffw-sh5-can-color-sort \
  --model-repo-id ggh-png/ffw-sh5-act-color-sort \
  --revision-tag v3.1.0 \
  --dry-run
```

`--dry-run`은 원격 저장소를 변경하지 않는다. 출력 manifest에서 episode 수, 필수
checkpoint, representation, normalization statistics와 SHA-256을 먼저 확인한다.

## 2. 업로드

실제 배포는 Hugging Face 로그인 후 명시적으로 수행한다. `--public`은 저장소 공개
설정까지 적용하므로 asset과 checkpoint의 재배포 권한을 먼저 확인해야 한다.

```bash
hf auth login
HF_XET_HIGH_PERFORMANCE=1 python scripts/publish_huggingface.py \
  --dataset-repo-id ggh-png/ffw-sh5-can-color-sort \
  --model-repo-id ggh-png/ffw-sh5-act-color-sort \
  --revision-tag v3.1.0 \
  --public
```

## 3. Tag 검증

업로드 뒤 두 저장소의 tag가 실제 main commit을 가리키는지 확인한다.

```bash
python - <<'PY'
from huggingface_hub import HfApi

api = HfApi()
for repo_id, repo_type in (
    ("ggh-png/ffw-sh5-can-color-sort", "dataset"),
    ("ggh-png/ffw-sh5-act-color-sort", "model"),
):
    main = api.repo_info(repo_id, repo_type=repo_type).sha
    tagged = api.repo_info(
        repo_id, repo_type=repo_type, revision="v3.1.0"
    ).sha
    print(repo_id, main, tagged, main == tagged)
PY
```

두 출력이 모두 `True`여야 한다. 새 코드 release와 자산 revision이 다르면 다운로드
문서, model card와 dataset card의 호환 버전을 함께 갱신한다.
