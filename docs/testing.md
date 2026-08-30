# 테스트와 검증

이 프로젝트는 창을 띄워 눈으로만 확인하지 않는다. 알고리즘 단위, kinematic 한 step,
실제 actuator/contact 물리를 서로 다른 gate로 반복 검증한다.

## 테스트 의존성 설치

저장소 전체의 Python 검사와 문서 빌드를 실행하려면 개발 의존성과 문서 의존성을
설치한다. `requirements-dev.txt`는 runtime, imitation, Hugging Face 의존성을 모두
포함하고, MkDocs 도구는 `requirements-docs.txt`로 분리되어 있다.

```bash
python -m pip install -r requirements-dev.txt -r requirements-docs.txt
```

기본 텔레오퍼레이션만 실행한다면 이 전체 구성이 필요하지 않다. 용도별 최소 설치는
[빠른 시작의 설치 프로필](getting-started.md#installation-profiles)을 참고한다.

## 어떤 테스트를 실행할까

| 변경한 영역 | 최소 테스트 | 릴리스 전 추가 테스트 |
|---|---|---|
| YAML 설정·로더 | `test_config.py` | 관련 알고리즘 테스트 |
| 문서만 | `mkdocs build --strict` | 내부 링크 검사 |
| UI/target/mode | `test_phase_6.py` | `test_whole_body.py` |
| IK/FK/kinematics | `test_phase_3.py`, `test_whole_body.py` | Phase 4, 6 |
| mobile/swerve | `test_phase_5.py`, `test_whole_body.py` | Phase 0–6 전체 |
| grasp/contact | `test_phase_1.py`, `test_phase_2.py` | Phase 3, 4 |
| collision CBF | `test_whole_body.py` | Phase 5, 6 |
| 모델 XML/actuator | 관련 Phase + `test_whole_body.py` | 전체 suite |
| imitation/ACT | `test_il_action.py`, `test_il_env.py`, `test_il_dataset.py` | `test_il_*.py` 전체 |
| 오른팔 모션 플래닝 | `test_planning_*.py` | [오른팔 모션 플래닝](guide/motion-planning.md) |

## 빠른 핵심 검증

```bash
python -m ruff check src scripts tests
MUJOCO_GL=osmesa python -m pytest -q
python -m mkdocs build --strict
```

Arm-only imitation 전체 gate:

```bash
for test_file in tests/test_il_*.py; do
  python3 "$test_file"
done
```

카메라 gate는 headless EGL을 사용하고, training gate는 3-frame synthetic episode로
실제 checkpoint/metric/PNG 산출물을 한 epoch 생성한다. Rerun SDK가 설치되지 않은
환경에서는 dependency boundary와 오류 안내를 검사하고, 실제 `.rrd` 생성은 SDK가
있는 학습·평가 환경에서 수행한다.

이 조합은 현재 UI target, Bimanual/Whole-body mode, ROS-free dependency, WBIK,
collision, 스워브와 실제 wheel-ground 추종을 빠르게 확인한다.

## 전체 suite

```bash
for p in 0 1 2 3 4 5 6; do
  python3 "tests/test_phase_${p}.py"
done
python3 tests/test_config.py
python3 tests/test_whole_body.py
mkdocs build --strict
```

pytest는 단위·통합 테스트를 한 번에 실행하는 기본 gate다. Phase 스크립트는 독립 실행도
지원하며 마지막 `PASS`와 exit code 0이 성공 기준이다. ROS launch test와 display
server는 필요하지 않고 CI 카메라는 headless OSMesa로 검증한다. NVIDIA/EGL이 준비된
로컬·서버 환경에서는 `MUJOCO_GL=egl`도 사용할 수 있다.

## Phase별 의미

| 파일 | 검증 범위 | 대표 성공 기준 |
|---|---|---|
| Phase 0 | 공식 모델 로드와 gravity-only 안정성 | 5초 동안 발산 없음 |
| Phase 1 | finger-can collision geometry | 20회 최대 관통 < 2 mm |
| Phase 2 | 고정 손 grasp + lift | 10회 중 8회 이상 성공 |
| Phase 3 | 오른팔 FK/Jacobian/IK/pick | IK 100개 ≥95%, pick ≥7/10 |
| Phase 4 | 전신 hold/양팔 IK/pick | hold drift 제한, IK/pick 회귀 |
| Phase 5 | keyboard, swerve FSM, 실제 물리 주행 | idle/전후/strafe/yaw/반전/충돌 통과 |
| Phase 6 | marker, gizmo, 숫자 task-space 입력, Bimanual, mode toggle | pose round-trip·IK 추종과 ON/OFF 불변성 |
| Whole-body | 명시적 box-QP, DLS 비용 변환, joint/collision CBF, rigid grasp, mobile WBIK | 수치/물리/latency 통합 gate |

## 3.0.0 모방학습 gate

- HDF5 schema 1.1의 16D qpos/qvel/action, 양쪽 EE pose, 세 카메라 길이 정렬
- reset마다 캔 색과 좌우 상자 색 배치를 독립 무작위화하고 올바른 target을 선택
- 같은 원본 episode를 Joint/Task 8D tensor로 변환하며 quaternion/frame 계약 검사
- checkpoint representation metadata와 UI `AUTO`/`JOINT`/`TASK` 필터 일치
- PTE 후보가 같은 미래 target timestep만 모으고 최신 예측에 가장 큰 가중치 적용
- Rerun Viewer가 종료되어도 flush 예외가 policy/teleop 종료를 다시 중단하지 않음
- Hugging Face manifest가 150 episode와 네 best checkpoint의 SHA-256을 기록

## 3.1.0 분석·구조 gate

- Joint/Task가 공용 dataset loader/trainer/config를 사용하고 representation strategy만 교체
- D97 Joint/Task와 D150 Joint/Task의 ordered episode split 동일성
- Grad-CAM이 policy camera별 finite heatmap, raw maximum과 gradient 크기를 반환
- continuous action scalar, signed target과 linear action direction의 gradient 경로
- Hugging Face dataset/model publish 계획과 선택적 Hub revision tag
- 연구 보고서의 25 Hz, 데이터 색상 구성과 PTE 수치가 manifest·평가 로그와 일치

## 핵심 제어 회귀가 증명하는 것

### Custom kinematics hard gate

`test_whole_body.py`는 앱 런타임의 FK/Jacobian 경로에서 다음 우회를 금지한다.

- `mj_forward`, MuJoCo site/point Jacobian API
- `data.site_xpos`, `data.site_xmat` 직접 참조
- `KinematicTree`/`DifferentialIKSolver` 내부의 `MjData` 생성

`test_phase_3.py`는 자체 FK pose를 독립 엔진 결과와 비교하고, 자체 Jacobian을
관절별 중앙 유한차분과 비교한다. 현재 최대 Jacobian 오차는 `2.33e-10`이다.

### Compact multi-viewport UI gate

UI 상태는 `control`, `diagnostics` 두 워크스페이스만 허용한다. 실제 GUI smoke에서는
MuJoCo 주 viewport 하나와 외부 플랫폼 viewport 두 개가 생성되고, Return/Detach에서
각각 1개/3개로 병합·분리되는지 확인한다.

### Arm-only hard gate

`WholeBodyIK.solve(..., whole_body_enabled=False)` 결과에서:

- base x/y/yaw와 lift `qdot`이 정확히 0
- 반환 `BodyTwist`가 정확히 zero
- 양팔 관절은 사용됨
- solver가 live qpos를 변경하지 않음
- 한 step 뒤 손 pose error가 감소

최근 검증에서는 combined error가 `89.4 mm → 56.4 mm`로 감소했다.

### Base participation gate

`test_whole_body.py`는 Whole-body ON 상태의 모바일 베이스 설정도 별도로 검사한다.

- `base_participation_scale=0.05`에서 base x/y/yaw가 축소된 속도 bound를 넘지 않음
- 기본 참여율보다 실제 base 속도 norm이 감소함
- `base_participation_scale=0.0`에서 base 3축 `qdot`이 정확히 0
- base를 꺼도 lift와 양팔 자유도는 계속 사용됨
- QP 해를 구한 뒤 `qdot[:3]`을 강제로 덮어쓰는 런타임 경로가 없음

### ON/OFF target 보존

`test_phase_6.py`는 다음을 일반 MoveL과 captured Bimanual MoveL에서 검사한다.

1. 전환 전 양손/virtual-object world pose 저장
2. ON→OFF 전환 뒤 position과 quaternion 비교
3. cached base command가 zero인지 검사
4. raw/smoothed target 동기화 검사
5. OFF→ON 왕복 뒤 같은 pose인지 검사
6. 실제 `_step_physics()`에서 OFF zero base와 수동 lift 적용 검사

### 키 해제 제동과 복귀 방지

물리 회귀는 1초 수동 후진 뒤:

- 차체 정지 < 0.5초
- 모든 바퀴 정지 < 0.5초
- 원래 방향 복귀 < 5 mm

최근 기준값은 차체 0.20초, 바퀴 0.32초, 복귀 3.73 mm다.

### Collision CBF

- 153개 monitored pair 구성과 scope
- 최근접점 distance gradient 대 중앙 유한차분
- self-collision 접근 속도가 분리 속도로 바뀌는지
- table 하강 명령의 CBF 위반 감소와 lift 방향 전환
- pair가 멀 때 기존 명령과 bit-level 동일성
- visualization이 controller와 같은 거리 데이터를 쓰는지

### 실제 Whole-body 물리 추종

베이스 qpos를 직접 쓰지 않고 wheel steer/drive actuator와 지면 contact로 다음 target을
추종한다.

| Trial | 확인하는 것 |
|---|---|
| longitudinal | 공통 전후 target과 base x 참여 |
| lateral | holonomic strafe와 base y 참여 |
| vertical | lift/팔 협력 |
| yaw | swerve 제자리 회전과 양손 자세 추종 |

## 결과를 해석하는 법

테스트는 “명령이 나왔는가”와 “물리가 실제로 따라갔는가”를 구분한다.

| 출력 종류 | 의미 |
|---|---|
| `gate: ... OK` | 알고리즘/상태 불변식 통과 |
| `error=A->B` | 한 step 또는 trial에서 목표 오차 감소 |
| `base=(x,y,yaw)` | 실제 물리 base 이동량 |
| `max|qacc|` | 발산/불안정성 감시 |
| `violation` | collision barrier 요구 위반량 |
| `ms/solve` | 25 Hz의 40 ms frame budget 대비 solver 시간 |

테스트 하나의 threshold를 낮춰 PASS로 만드는 대신, 실패한 물리/수치 원인을 먼저
분리한다. 예를 들어 base가 안 움직이면 solver command, swerve wheel command,
wheel-ground contact를 차례로 나눠 본다.

## 문서 검증

```bash
mkdocs build --strict
```

추가로 repository 내부 Markdown 링크가 실제 파일과 anchor를 가리키는지 검사한다.
문서에 상수값을 적을 때는 코드의 기본값과 테스트 출력도 함께 확인한다.

## 릴리스 체크리스트

1. 의도한 branch인지 확인
2. `src/imgui.ini` 같은 개인 runtime 상태 제외
3. Phase 0–6 + whole-body 전체 통과
4. `mkdocs build --strict` 통과
5. CHANGELOG와 release 문서 갱신
6. 커밋과 branch push
7. annotated semver tag push
8. GitHub release가 같은 commit을 가리키는지 검증
