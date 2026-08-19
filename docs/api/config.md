# 설정 API

대상 모듈은 `ffw_sh5_grasp.config`다.

## `load_settings(path=None)`

기본 `config/default.yaml`과 선택한 사용자 YAML을 병합하고 형식을 검증해 `Settings`를
반환한다.

| 항목 | 내용 |
|---|---|
| `path` | 사용자 YAML 경로. 생략하면 `FFW_SH5_CONFIG`, 그마저 없으면 기본 YAML 사용 |
| 반환 | 검증된 `Settings` |
| 허용 | 기본 YAML의 일부 키만 덮어쓰기 |
| 오류 | 없는 파일, YAML 문법, 알 수 없는 키, 자료형·목록 길이·schema 불일치 |

```python
from ffw_sh5_grasp.config import load_settings

settings = load_settings("config/local.yaml")
damping = settings.number("whole_body_ik.solver.dls_damping", positive=True)
```

## `Settings`

검증된 값을 깊은 복사로 보관한다. 실제로 읽은 파일은 `settings.path`에서 확인한다.

| 메서드 | 반환 | 검사 |
|---|---|---|
| `get(dotted_path)` | 값의 복사본 | 경로가 없으면 `KeyError` |
| `number(dotted_path, *, minimum=None, positive=False)` | `float` | 숫자 여부와 선택한 범위 |
| `integer(dotted_path, *, minimum=None)` | `int` | 정수 여부와 선택한 최솟값 |

모듈 전역 `SETTINGS`는 import 시 `load_settings()`로 생성된다. 사용자 YAML 적용 순서는
[YAML 파라미터 설정](../configuration.md)을 참고한다.
