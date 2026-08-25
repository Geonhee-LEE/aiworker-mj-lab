"""저장소 안에서 공유하는 모델·루트 경로."""

from pathlib import Path

from .config import SETTINGS

REPO_ROOT = Path(__file__).resolve().parents[2]
_configured_model_path = Path(SETTINGS.get("application.model_path"))
MODEL_PATH = (
    _configured_model_path
    if _configured_model_path.is_absolute()
    else REPO_ROOT / _configured_model_path
)
