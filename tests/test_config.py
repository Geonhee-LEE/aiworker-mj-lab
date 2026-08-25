"""YAML 설정의 기본값, 부분 덮어쓰기와 오탈자 검증을 확인한다."""

import ast
import json
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = REPO_ROOT / "src"
DEFAULT_CONFIG = REPO_ROOT / "config" / "default.yaml"
sys.path.insert(0, str(SRC_DIR))

from ffw_sh5_grasp.config import load_settings  # noqa: E402


def _write(path, content):
    """설정 회귀에서 사용할 임시 YAML 문자열을 UTF-8 파일로 기록한다."""
    path.write_text(content, encoding="utf-8")


def _leaf_paths(value, prefix=""):
    """중첩 YAML에서 실제 설정값이 놓인 점 구분 경로를 모두 반환한다."""
    if isinstance(value, dict):
        paths = []
        for key, child in value.items():
            child_prefix = f"{prefix}.{key}" if prefix else str(key)
            paths.extend(_leaf_paths(child, child_prefix))
        return paths
    return [prefix]


def _settings_access_paths():
    """소스의 ``SETTINGS`` 접근에서 리터럴 경로와 하위 매핑 사용을 수집한다."""
    paths = {"schema_version"}
    for source_path in SRC_DIR.rglob("*.py"):
        tree = ast.parse(
            source_path.read_text(encoding="utf-8"), filename=str(source_path)
        )
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not isinstance(
                node.func, ast.Attribute
            ):
                continue
            owner = node.func.value
            if not (
                isinstance(owner, ast.Name) and owner.id == "SETTINGS" and node.args
            ):
                continue
            path_node = node.args[0]
            if isinstance(path_node, ast.Constant) and isinstance(path_node.value, str):
                paths.add(path_node.value)
    return paths


def main():
    """기본값·부분 override·오류 검증과 YAML 한국어 주석 존재 여부를 검사한다."""
    defaults = load_settings()
    assert defaults.path == DEFAULT_CONFIG
    assert defaults.number("arm_control.proportional_gain") == 600.0
    assert defaults.get("base.geometry.wheel_positions_m")["rear_wheel"] == [
        -0.2899,
        0.0,
    ]

    # 사용자가 일부 값만 적어도 나머지는 기본 YAML에서 상속되어야 한다.
    with tempfile.TemporaryDirectory() as directory:
        override_path = Path(directory) / "custom.yaml"
        _write(
            override_path,
            "# 시험용 부분 설정이다.\n"
            "arm_control:\n"
            "  proportional_gain: 321.0\n"
            "base:\n"
            "  teleop:\n"
            "    cruise_speed_m_s: 0.4\n"
            "    max_speed_m_s: 0.5\n",
        )
        custom = load_settings(override_path)
        assert custom.number("arm_control.proportional_gain") == 321.0
        assert custom.number("arm_control.derivative_gain") == 40.0

        # 모듈 import 시 선택한 파일의 값이 실제 공개 기본값으로 연결되는지 확인한다.
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(SRC_DIR)
        environment["FFW_SH5_CONFIG"] = str(override_path)
        output = subprocess.check_output(
            [
                sys.executable,
                "-c",
                "import json; "
                "from ffw_sh5_grasp.control import arm, base; "
                "print(json.dumps([arm.DEFAULT_KP, base.K_SPEED, base.K_MAX]))",
            ],
            cwd=REPO_ROOT,
            env=environment,
            text=True,
        )
        assert json.loads(output) == [321.0, 0.4, 0.5]

        typo_path = Path(directory) / "typo.yaml"
        _write(typo_path, "arm_control:\n  unknown_gain: 1.0\n")
        try:
            load_settings(typo_path)
        except ValueError as error:
            assert "알 수 없는 설정 키" in str(error)
        else:
            raise AssertionError("오탈자 설정 키를 허용했습니다.")

    # 기본 YAML의 사람이 읽는 주석은 모두 한국어 설명을 포함해야 한다.
    comment_lines = [
        line.strip()[1:].strip()
        for line in DEFAULT_CONFIG.read_text(encoding="utf-8").splitlines()
        if line.strip().startswith("#")
    ]
    assert comment_lines
    assert all(re.search(r"[가-힣]", comment) for comment in comment_lines)
    # 설정값을 추가하고 읽지 않는 실수를 막는다. 매핑 전체를 get한 경우 하위 leaf도
    # 소비된 것으로 보며 schema_version은 설정 로더가 직접 사용한다.
    default_data = yaml.safe_load(DEFAULT_CONFIG.read_text(encoding="utf-8"))
    access_paths = _settings_access_paths()
    unused_paths = [
        path
        for path in _leaf_paths(default_data)
        if not any(path == used or path.startswith(f"{used}.") for used in access_paths)
    ]
    assert not unused_paths, f"코드에서 사용하지 않는 YAML 설정: {unused_paths}"
    print(
        f"YAML 설정 기본값/덮어쓰기/검증/한국어 주석/사용 여부: "
        f"{len(_leaf_paths(default_data))}개 OK"
    )
    print("PASS")


if __name__ == "__main__":
    main()
