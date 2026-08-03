"""한국어 주석 YAML에서 실행·제어 설정을 읽고 검증한다."""

from copy import deepcopy
import math
import os
from pathlib import Path

try:
    import yaml
except ImportError as error:  # pragma: no cover - 설치 오류 안내 경로
    raise RuntimeError(
        "YAML 설정을 읽으려면 PyYAML이 필요합니다: pip install pyyaml"
    ) from error


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_PATH = REPO_ROOT / "config" / "default.yaml"
CONFIG_ENV_VAR = "FFW_SH5_CONFIG"


def _read_yaml(path):
    """UTF-8 YAML 파일을 매핑으로 읽고 파일·문법·최상위 형식 오류를 설명한다."""
    try:
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
    except FileNotFoundError as error:
        raise FileNotFoundError(f"설정 파일을 찾을 수 없습니다: {path}") from error
    except yaml.YAMLError as error:
        raise ValueError(f"YAML 문법 오류가 있습니다({path}): {error}") from error
    if not isinstance(data, dict):
        raise ValueError(f"YAML 최상위 값은 매핑이어야 합니다: {path}")
    return data


def _merge_known(defaults, overrides, prefix=""):
    """기본 스키마에 사용자 값을 병합하고 알 수 없는 키는 거부한다."""
    result = deepcopy(defaults)
    for key, value in overrides.items():
        path = f"{prefix}.{key}" if prefix else str(key)
        if key not in defaults:
            raise ValueError(f"알 수 없는 설정 키입니다: {path}")
        default = defaults[key]
        if isinstance(default, dict):
            if not isinstance(value, dict):
                raise TypeError(f"설정 {path}는 매핑이어야 합니다.")
            result[key] = _merge_known(default, value, path)
        else:
            result[key] = deepcopy(value)
    return result


def _validate_shape(default, value, path):
    """기본 YAML과 같은 자료 구조인지 재귀적으로 검사한다."""
    if isinstance(default, dict):
        if not isinstance(value, dict):
            raise TypeError(f"설정 {path}는 매핑이어야 합니다.")
        for key, child in default.items():
            _validate_shape(child, value[key], f"{path}.{key}" if path else key)
        return
    if isinstance(default, list):
        if not isinstance(value, list) or len(value) != len(default):
            raise TypeError(f"설정 {path}는 길이 {len(default)}의 목록이어야 합니다.")
        for index, (default_item, item) in enumerate(zip(default, value)):
            _validate_shape(default_item, item, f"{path}[{index}]")
        return
    if isinstance(default, bool):
        if not isinstance(value, bool):
            raise TypeError(f"설정 {path}는 참/거짓 값이어야 합니다.")
        return
    if isinstance(default, (int, float)) and not isinstance(default, bool):
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise TypeError(f"설정 {path}는 숫자여야 합니다.")
        if not math.isfinite(float(value)):
            raise ValueError(f"설정 {path}는 유한한 숫자여야 합니다.")
        return
    if not isinstance(value, type(default)):
        raise TypeError(f"설정 {path}의 자료형이 올바르지 않습니다.")


class Settings:
    """점으로 구분한 경로로 읽는 불변 설정 스냅샷."""

    def __init__(self, data, path):
        """검증된 설정 매핑을 깊은 복사하고 원본 YAML 경로를 함께 보관한다."""
        self._data = deepcopy(data)
        self.path = Path(path)

    def get(self, dotted_path):
        """점으로 구분한 키 경로의 값을 찾아 호출자가 바꿀 수 없는 복사본으로 반환한다."""
        value = self._data
        for key in dotted_path.split("."):
            if not isinstance(value, dict) or key not in value:
                raise KeyError(f"설정 키가 없습니다: {dotted_path}")
            value = value[key]
        return deepcopy(value)

    def number(self, dotted_path, *, minimum=None, positive=False):
        """설정값을 실수로 읽고 양수 여부와 선택적 최솟값을 검증한다."""
        value = self.get(dotted_path)
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise TypeError(f"설정 {dotted_path}는 숫자여야 합니다.")
        result = float(value)
        if positive and result <= 0.0:
            raise ValueError(f"설정 {dotted_path}는 0보다 커야 합니다.")
        if minimum is not None and result < minimum:
            raise ValueError(f"설정 {dotted_path}는 {minimum} 이상이어야 합니다.")
        return result

    def integer(self, dotted_path, *, minimum=None):
        """설정값을 정수로 읽고 선택적 최솟값 조건을 검증한다."""
        value = self.get(dotted_path)
        if not isinstance(value, int) or isinstance(value, bool):
            raise TypeError(f"설정 {dotted_path}는 정수여야 합니다.")
        if minimum is not None and value < minimum:
            raise ValueError(f"설정 {dotted_path}는 {minimum} 이상이어야 합니다.")
        return value


def load_settings(path=None):
    """기본 설정에 선택한 사용자 YAML을 병합해 반환한다.

    ``path``가 없으면 ``FFW_SH5_CONFIG`` 환경 변수를 확인한다. 사용자 파일은 일부
    항목만 적어도 되며, 기본 파일에 없는 키는 오탈자로 간주해 거부한다.
    """
    defaults = _read_yaml(DEFAULT_CONFIG_PATH)
    selected = path or os.environ.get(CONFIG_ENV_VAR)
    if selected is None:
        data = defaults
        source = DEFAULT_CONFIG_PATH
    else:
        source = Path(selected).expanduser()
        if not source.is_absolute():
            source = (Path.cwd() / source).resolve()
        data = _merge_known(defaults, _read_yaml(source))
    _validate_shape(defaults, data, "")
    if data["schema_version"] != defaults["schema_version"]:
        raise ValueError(
            f"지원하지 않는 설정 schema_version입니다: {data['schema_version']}"
        )
    return Settings(data, source)


SETTINGS = load_settings()


__all__ = [
    "CONFIG_ENV_VAR",
    "DEFAULT_CONFIG_PATH",
    "SETTINGS",
    "Settings",
    "load_settings",
]
