# 설정 API

대상 모듈은 `ffw_sh5_grasp.config`다. 기본 YAML과 선택적 사용자 YAML을 시작할 때
한 번 병합하고, 이후 계산 모듈에는 검증된 설정 스냅샷을 제공한다.

## `load_settings(path=None)`

**직관:** 작은 사용자 YAML을 기본 설정 위에 덮어써서, 오탈자까지 검사된 설정
한 벌을 만든다.

- **입력:** 사용자 YAML 경로. 생략하면 `FFW_SH5_CONFIG` 환경 변수를 확인하고,
  그것도 없으면 `config/default.yaml`만 사용한다.
- **반환:** `Settings`. 사용자 파일은 기본 YAML의 일부 항목만 포함해도 된다.
- **오류:** 알 수 없는 키, 자료형·목록 길이 불일치, 비유한 숫자, schema version
  불일치는 시작 단계에서 거부한다.
- **사용 시점:** 독립 도구나 테스트에서 앱과 같은 규칙으로 설정을 읽을 때 사용한다.

```python
settings = load_settings("config/local.yaml")
damping = settings.number("kinematics.damping", positive=True)
```

## `Settings`

검증이 끝난 설정의 읽기 전용 스냅샷이다. 반환값을 복사해서 주므로 호출자가 내부
설정을 우연히 바꿀 수 없다. 실제로 사용한 YAML 경로는 `settings.path`에 있다.

### `Settings.get(dotted_path)`

- **직관:** 중첩된 YAML을 `"whole_body_ik.velocity_limits.arm"` 같은 한 줄
  경로로 읽는다.
- **입력:** 점으로 구분한 키 경로.
- **반환:** 값의 복사본. 목록·매핑을 수정해도 원본 설정에는 영향이 없다.
- **오류:** 경로가 없으면 `KeyError`.

### `Settings.number(dotted_path, minimum=None, positive=False)`

- **직관:** gain·속도·거리처럼 실수가 필요한 값을 읽으며 범위도 함께 검사한다.
- **입력:** 키 경로, 선택적 최솟값, 양수 강제 여부.
- **반환:** `float`.
- **오류:** boolean을 포함한 비숫자, 최솟값 위반, 0 이하의 양수 항목.

### `Settings.integer(dotted_path, minimum=None)`

- **직관:** 반복 횟수·창 크기처럼 정수여야 하는 값을 읽는다.
- **입력:** 키 경로와 선택적 최솟값.
- **반환:** `int`.
- **오류:** 실수·boolean·최솟값 위반.

전역 `SETTINGS`는 모듈 import 시 `load_settings()`로 만들어진 기본 스냅샷이다.
CLI에서 `--config`를 넘기면 앱이 제어 모듈을 import하기 전에 환경 변수로 경로를
전달한다. 적용 순서는 [YAML 파라미터 설정](../configuration.md)에 설명한다.
