# YAML 파라미터 설정

실행과 알고리즘 튜닝 값은 저장소의 `config/default.yaml`에 모여 있다.
이 파일에는 각 값의 의미와 단위를 설명하는 한국어 주석이 함께 들어 있다. 코드에서
숫자를 직접 바꾸기보다 사용자 설정 파일을 만들고 필요한 항목만 덮어쓰는 방식을 권장한다.

## 설정 범위

| YAML 구역 | 적용 코드 | 주요 값 |
|---|---|---|
| `application` | `application/teleop.py`, `paths.py` | 모델, 창 크기, 제어 주기, 목표 변화율, 리프트·파지 입력 |
| `arm_control` | `control/arm.py` | 팔 PD 이득 |
| `kinematics` | `kinematics/solver.py` | DLS, 반복 횟수, 허용 오차, 다중 시작 |
| `whole_body_ik` | `control/whole_body.py` | 작업 가중치, 속도 한계, 관절·충돌 CBF |
| `optimization` | `control/optimization.py` | BVLS 수치 허용 오차와 반복 상한 |
| `base` | `control/base.py` | 키보드 속도, 스워브 형상, 조향·반전 제어 |
| `grasp` | `control/grasp.py` | 손 시너지 각도·비율과 파지 판정 임계값 |
| `ui` | `visualization/ui.py` | 조그 간격, 슬라이더 범위, 창 배치 |
| `render` | `visualization/render.py` | 카메라, 기즈모, 충돌 표시 |

관절·몸체·사이트 이름과 행렬 차원은 모델 인터페이스이므로 튜닝값이 아니다. MuJoCo의
질량, 관성, 접촉 형상과 액추에이터 정의는 MJCF 모델 자체가 원본이므로
`models/*.xml`에서 관리한다. YAML은 그 모델을 사용하는 런타임과 알고리즘의 조절값을
담는다.

테스트의 성공률·오차 허용치도 YAML로 옮기지 않는다. 실행 설정과 합격 기준을 함께
바꾸면 실패를 숨길 수 있으므로, `tests/`의 회귀 기준은 독립된 검증 계약으로 유지한다.

## 기본값 그대로 실행

별도 인자를 주지 않으면 `config/default.yaml`을 읽는다.

```bash
python3 src/teleop_app.py
```

YAML 로더는 `PyYAML`을 사용한다.

```bash
python -m pip install pyyaml
```

## 사용자 설정 만들기

사용자 파일에는 바꾸려는 값만 적으면 된다. 나머지는 기본 설정에서 상속된다.

```yaml
# config/local.yaml
# 팔을 조금 부드럽게 하고 수동 주행 속도를 낮추는 예시다.
arm_control:
  proportional_gain: 480.0
  derivative_gain: 36.0

base:
  teleop:
    cruise_speed_m_s: 0.40
    max_speed_m_s: 0.50

whole_body_ik:
  collision_safe_distance_m: 0.015
```

앱 실행 시 `--config`로 선택한다.

```bash
python3 src/teleop_app.py --config config/local.yaml
```

테스트나 별도 Python 프로그램에서는 설정 의존 모듈을 import하기 전에 환경 변수를
지정한다.

```bash
FFW_SH5_CONFIG=config/local.yaml python3 tests/test_phase_6.py
```

```python
import os

# 제어 모듈을 불러오기 전에 사용자 설정 경로를 지정한다.
os.environ["FFW_SH5_CONFIG"] = "config/local.yaml"

from ffw_sh5_grasp.control import base
```

## 검증 규칙

설정 로더는 다음 오류를 시작 단계에서 차단한다.

- 기본 설정에 없는 키: 오탈자로 보고 오류
- 숫자 대신 문자열을 넣는 등 자료형 불일치
- 목록 길이 불일치
- `NaN`, 무한대 같은 유한하지 않은 숫자
- 음수가 될 수 없는 이득·거리·속도
- 최솟값과 최댓값 순서가 뒤집힌 범위
- 충돌 `buffer`가 `safe_distance`보다 작거나 같은 설정

기본 파일을 직접 수정할 수는 있지만, 비교와 복구가 쉬운 부분 덮어쓰기 파일을 별도로
두는 편이 안전하다. 설정 동작과 한국어 주석은 `tests/test_config.py`가 회귀 검사한다.

## 값 변경 후 검증

제어값은 서로 영향을 주므로 최소한 다음 검사를 실행한다.

```bash
python3 tests/test_config.py
python3 tests/test_phase_5.py
python3 tests/test_phase_6.py
python3 tests/test_whole_body.py
```

특히 스워브 형상·속도 제한, 충돌 거리, 팔 PD, 전신 IK 가중치를 바꿨다면 개별 단위
검사만으로 끝내지 말고 실제 물리 회귀까지 확인한다.
