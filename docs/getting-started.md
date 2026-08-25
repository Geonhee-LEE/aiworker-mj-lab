# 10분 빠른 시작

이 문서는 저장소를 처음 받은 사람이 앱을 실행하고, 베이스와 한 손을 움직이고,
전신 제어 ON/OFF 차이를 확인하는 데 필요한 내용만 담는다.

## 1. 준비 사항과 저장소 받기

- Linux 데스크톱과 OpenGL을 사용할 수 있는 화면 세션
- Python 3.12와 `venv` (Ubuntu 24.04, Python 3.12에서 검증)
- Git

Ubuntu 24.04에서는 먼저 Python 가상환경과 headless OpenGL 런타임을 설치한다.

```bash
sudo apt-get update
sudo apt-get install --yes python3.12-venv libgl1 libosmesa6
```

저장소를 받은 뒤 모든 명령을 저장소 루트에서 실행한다.

```bash
git clone https://github.com/ggh-png/aiworker-mj-lab.git
cd aiworker-mj-lab
```

현재 앱은 주 GLFW 창에 MuJoCo 3D 화면을 띄우고, ImGui multi-viewport로 기능별
패널을 별도 OS 창에 띄운다. ROS2 workspace, `colcon`, MoveIt, controller manager는
필요하지 않다.

## 2. 가상환경과 설치

시스템 Python을 직접 수정하지 않도록 가상환경을 권장한다.

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-runtime.txt
```

`requirements-runtime.txt`에는 일반 teleop 실행에 필요한 MuJoCo, GLFW, ImGui,
NumPy, YAML과 앱의 ACT checkpoint 탐색 경로가 사용하는 HDF5 의존성이 포함된다.
시스템 Python을 직접 수정하는 방식은 권장하지 않는다.

설치 확인:

```bash
python -c "import glfw, h5py, mujoco, numpy, yaml; from imgui_bundle import imgui; print('runtime imports OK')"
```

ACT 학습·평가 또는 공개 checkpoint와 dataset 다운로드까지 사용할 때는 같은
가상환경에 모방학습/Hugging Face 의존성을 추가한다.

```bash
python -m pip install -r requirements-huggingface.txt
```

공개 정책을 바로 실행하거나 공개 HDF5로 학습하는 절차는
[정책·데이터셋 다운로드](huggingface.md)를 따른다. 직접 데이터를 기록하고 학습하려면
[모방학습 명령어](imitation-commands.md)로 이동한다.

## 3. 먼저 headless 검증

창을 띄우기 전에 모델과 핵심 알고리즘이 동작하는지 확인한다.

```bash
python tests/test_phase_6.py
python tests/test_whole_body.py
```

마지막 줄이 각각 `PASS`면 marker/UI 상태와 whole-body/mobile/collision 알고리즘이
정상이다. 이 테스트는 화면이 없어도 실행된다.

## 4. 앱 실행

```bash
python src/teleop_app.py
```

속도, IK·제어 이득, 파지와 UI 범위를 바꾸려면 코드를 수정하지 말고
[YAML 파라미터 설정](configuration.md)에 따라 사용자 설정을 적용한다.

정상이라면 주 창에는 3D 장면과 상태 창이 보이고, 두 워크스페이스가 주 창 오른쪽
바깥의 별도 OS 창으로 보인다.

- 3D 장면: 로봇, table, can, 손 목표 marker와 gizmo
- `Status & Windows`: 상태와 다른 창의 표시 여부
- `Control Center`: Target, Right Arm, Left Arm, Robot/Grasp 탭
- `Diagnostics`: Kinematic Tree, Joint Monitor 탭

창이 열리지 않으면 바로 [문제 해결의 창/그래픽 항목](troubleshooting.md#window-startup)으로
이동한다.

## 5. 첫 조작: 베이스

1. 마우스로 3D 화면을 한 번 클릭해 창에 키보드 focus를 준다.
2. `Up`을 1초 정도 누르면 로봇이 전진한다.
3. 키를 놓고 바퀴와 차체가 제동하는 것을 확인한다.
4. `[`와 `]`로 strafe, `Left`와 `Right`로 제자리 yaw를 확인한다.

!!! note "키를 놓은 직후"
    목표 속도는 zero로 바뀌지만 물리 차체는 순간 정지하지 않는다. 정상 회귀에서는
    차체가 약 0.20초, 모든 wheel joint가 약 0.32초 안에 정지한다. 0.5초 이상 계속
    구르거나 반대 방향으로 크게 돌아오면 [모바일 문제 해결](troubleshooting.md#wheel-keeps-rolling)을 본다.

## 6. 첫 조작: 오른손 MoveL

1. `Control Center → Target` 탭에서 controller가 `MoveL`인지 확인한다.
2. marker를 `Right goal`로 선택한다.
3. Position jog의 `X+`를 몇 번 누르거나 3D gizmo의 X 화살표를 조금 끈다.
4. 상태 창 또는 `Right Arm` 탭에서 IK error가 줄어드는지 확인한다.

한 번에 큰 값을 주면 앱이 frame당 최대 3 cm/8°로 target을 ramp한다. marker가 먼저
가고 실제 손이 뒤따라오는 것은 정상이다.

## 7. Whole-body ON/OFF 비교

버튼은 `Lift / Utilities` 맨 위에 있다.

=== "ON"

    - 손 목표를 따라 base, lift, IK 상태의 팔이 함께 움직일 수 있다.
    - 양손의 공통 이동이 크면 base가 적극적으로 참여한다.
    - 상태줄에 `Whole-body IK ON`이 표시된다.

=== "OFF (arm-only)"

    - base x/y/yaw와 lift의 **IK 속도만** 정확히 0으로 고정한다.
    - 팔이 도달 가능한 범위에서 팔만 목표를 추종한다.
    - 키보드 base 주행과 `Q/E` 또는 lift slider는 계속 동작한다.
    - 상태줄에 `Whole-body IK OFF (arm-only)`가 표시된다.

버튼을 누르는 순간 손과 virtual-object의 world 목표는 보존된다. 목표 marker가 다른
위치로 튀거나 이전 base 명령이 다시 재생되면 정상 동작이 아니다.

## 8. Collision 표시 확인

`V`를 누르거나 **Collision CBF Viz**를 체크한다.

| 표시 | 의미 |
|---|---|
| 반투명 파랑 geometry | 제어기가 충돌 거리 계산에 사용하는 형상 |
| 노랑 선 | 1~3 cm 감시 구간 |
| 주황 선 | 0~1 cm 안전거리 안쪽 |
| 빨강 선 | signed distance가 음수인 관통 상태 |

이 표시는 물리 contact(`G`)와 다르다. `V`는 예방 제어용 거리, `G`는 이미 발생한
물리 접촉점과 힘을 보여준다.

## 9. 종료와 다음 문서

창을 닫아 종료한다. 다음에는 목적에 따라 이동한다.

- 모드 조합이 헷갈리면 [모드 선택](control-modes.md)
- 모든 버튼과 키를 보려면 [화면과 조작](run.md)
- 왜 이런 구조인지 이해하려면 [시스템 구조](overview.md)
- 이상 동작을 진단하려면 [문제 해결](troubleshooting.md)
