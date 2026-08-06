# YAML 파라미터 설정

실행과 알고리즘 튜닝 값은 저장소의 `config/default.yaml`에 모여 있다.
이 파일에는 각 값의 의미와 단위를 설명하는 한국어 주석이 함께 들어 있다. 코드에서
숫자를 직접 바꾸기보다 사용자 설정 파일을 만들고 필요한 항목만 덮어쓰는 방식을 권장한다.

## 설정 범위

| YAML 구역 | 적용 코드 | 주요 값 |
|---|---|---|
| `application` | `application/teleop.py`, `paths.py` | 모델, 창 크기, 제어 주기, 목표 변화율, 리프트·파지 입력 |
| `arm_control` | `control/arm.py` | 팔 PD 이득 |
| `whole_body_ik` | `control/whole_body.py`, `kinematics/solver.py` | 해법 선택, 작업 가중치, 속도 한계, 관절·충돌 CBF |
| `base` | `control/base.py` | 키보드 속도, 스워브 형상, 조향·반전 제어 |
| `grasp` | `control/grasp.py` | 손 시너지 각도·비율과 파지 판정 임계값 |
| `ui` | `visualization/ui.py` | 조그 기본값과 창 배치 |
| `render` | `visualization/render.py` | 카메라 프리셋 |

관절·몸체·사이트 이름과 행렬 차원은 모델 인터페이스이므로 튜닝값이 아니다. MuJoCo의
질량, 관성, 접촉 형상과 액추에이터 정의는 MJCF 모델 자체가 원본이므로
`models/*.xml`에서 관리한다. YAML은 그 모델을 사용하는 런타임과 알고리즘의 조절값을
담는다.

QP 반복 상한, DLS backtracking 규칙, UI slider 안전 범위와 렌더링 색상처럼 구현과
함께 검증되어야 하는 값은 YAML 파라미터로 취급하지 않는다. 자주 조절하는 로봇 동작,
물리 기하, 목표와 화면 배치만 YAML에 남겨 설정 파일이 알고리즘 내부 상수 목록으로
변하지 않도록 한다.

## 스키마 5 마이그레이션

사용하지 않던 반복형 `solve_pose` 경로와 함께 최상위 `kinematics` 설정 구역을
삭제했다. 실행 중 선택하는 DLS damping은 계속
`whole_body_ik.solver.dls_damping`에서 설정한다. 사용자 YAML에 `schema_version: 4`나
옛 `kinematics` 키가 있다면 해당 항목을 제거하고 버전을 5로 올린다.

## 스키마 4 마이그레이션

QP의 손 위치·방향, rigid grasp, 공통 base와 collision slack까지 모든 잔차를 대응하는
속도 상한으로 나눈 무차원 값으로 통일했다.

\[
J_k=s_k\left(\frac{r_k}{v_{scale,k}}\right)^2
\]

이제 UI에 보이는 모든 값은 단위 없는 `strength`다. `1.0`은 해당 잔차가 대표 속도에
도달했을 때 비용 1이라는 뜻이다. 기존 raw 비용을 유지하는 변환은
`strength = old_weight * speed_scale²`이다.

| 목적함수 | 스키마 3 raw weight | 속도 scale | 스키마 4 strength |
|---|---:|---:|---:|
| 손 position | 10 | 1.2 m/s | 14.4 |
| 손 orientation | 5 | 3.0 rad/s | 45 |
| rigid grasp position | 250 | 1.2 m/s | 360 |
| rigid grasp orientation | 250 | 3.0 rad/s | 2250 |
| common base translation | 30 | 0.55 m/s | 9.075 |
| common base yaw | 100 | 1.4 rad/s | 196 |

단일 `rigid_grasp_weight` 키는 단위가 다른 병진·회전을 분리하기 위해
`rigid_grasp_weights.position`과 `.orientation`으로 바뀌었다. Collision safety
projection은 기준 명령을 각 자유도의 속도 상한으로, CBF 위반량을 최대 task 선속도로
나눈다. `collision_slack_weight` 기본값 1000은 유지했지만 이제 무차원 slack 비용이다.

## 스키마 3 마이그레이션

QP의 `damping_weights`와 `posture_weights`는 raw 속도 비용에서 속도 상한 기준의
무차원 strength로 바뀌었다.

\[
J_{regularization}
= \sum_i s_i\left(\frac{\dot q_i}{v_{max,i}}\right)^2
\]

스키마 2의 동일한 동작을 유지하려면 `strength = old_weight * velocity_limit²`로
변환한다. 기본 설정은 이 변환을 적용했으므로 업그레이드 전후 기본 명령은 같다.

| 자유도 | 스키마 2 raw weight | 스키마 3 strength |
|---|---:|---:|
| base linear | 0.25 | 0.075625 |
| base yaw | 0.20 | 0.392 |
| lift damping | 0.12 | 0.0147 |
| arm damping | 0.045 | 0.91125 |
| lift posture | 0.10 | 0.01225 |
| arm posture | 0.025 | 0.50625 |

UI의 damping/posture slider는 `1e-4~1e3` 로그 범위를 사용한다. `1.0`은 해당
자유도의 속도 상한에 도달했을 때 비용 1이라는 뜻이다. 정확히 0으로 끄려면 YAML이나
`set_qp_weight()`를 사용한다.

## 스키마 2 마이그레이션

중복 파라미터를 합치면서 기본 설정의 leaf 필드는 152개에서 124개로 줄었다. 기존
사용자 YAML에서 다음 키를 사용했다면 새 형식으로 바꾼다.

| 이전 키 | 현재 키 또는 처리 |
|---|---|
| `whole_body_ik.base.enabled: false` | `base.participation_scale: 0.0` |
| `velocity_limits.base_x`, `base_y` | 공통 `velocity_limits.base_linear` |
| `velocity_limits.lift_joint`, `arm_default` | `velocity_limits.lift`, `arm` |
| `damping_weights.base_lift: [x, y, yaw, lift]` | `base_linear`, `base_yaw`, `lift`로 분리 |
| `posture_weights.base_lift` | 0이 아닌 lift 값만 `posture_weights.lift`에 지정 |
| `common_base.task_weights: [x, y, yaw]` | `translation`, `yaw` 매핑 |
| `optimization.*` | 검증된 QP 구현 상수로 이동 |
| UI range·렌더링 스타일 키 | 고정된 시각화 구현 상수로 이동 |

x/y에 서로 다른 값을 사용하던 사용자 설정은 더 보수적인 큰 비용 또는 작은 속도
상한을 `base_linear`에 선택한다. 현재 로봇의 평면 이동은 등방성으로 운용하므로 기본
설정에서는 x/y를 별도로 조절하지 않는다.

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
  solver:
    # pseudoinverse, dls, qp 중 시작 해법을 고른다.
    method: dls
    dls_damping: 0.08
  collision_safe_distance_m: 0.015
  # 베이스를 거의 고정하되 lift와 양팔은 계속 전신 QP에 참여시킨다.
  base:
    participation_scale: 0.05
```

앱 실행 시 `--config`로 선택한다.

```bash
python3 src/teleop_app.py --config config/local.yaml
```

베이스의 자동 IK 명령을 정확히 0으로 고정하려면 다음처럼 설정한다. 이 옵션은 lift를
막지 않는다는 점에서 UI의 Whole-body OFF와 다르다.

```yaml
whole_body_ik:
  base:
    participation_scale: 0.0
```

`participation_scale`은 `0.0`~`1.0` 범위이며 명시적 공통 base 목표와 x/y/yaw 속도
상한에 함께 곱해진다. `0.05`이면 기본 상한 0.55 m/s, 0.55 m/s, 1.4 rad/s가 각각
0.0275 m/s, 0.0275 m/s, 0.07 rad/s가 된다. base를 더 비싼 QP 선택지로만 만들고
속도 상한은 유지하려면 이 값 대신 `damping_weights.base_linear`와
`damping_weights.base_yaw`를 높인다.

앱 실행 중에는 **IK Solver** 탭에서 해법을 즉시 바꿀 수 있다. QP를 선택하면 위치,
자세, rigid grasp 위치·방향, base/lift/arm damping, posture와 collision CBF slack의
무차원 strength가 표시된다. 각 슬라이더 위에 마우스를 올리면 목적과 정규화 기준이
나온다. UI 변경은 현재 프로세스에만 적용되며 다음 실행 기본값은 YAML로 관리한다.

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
