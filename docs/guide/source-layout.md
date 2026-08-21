# 소스 구조와 정리 원칙

`src/` 루트는 실행 진입점만 둔다. 알고리즘, 상태, UI 구현을 루트 스크립트에 넣지
않는다.

저장소 전체 트리와 개별 파일 책임은
[프로젝트 파일 트리와 역할](project-tree.md)에서 한 번에 확인할 수 있다.

```text
src/
├── teleop_app.py                 # teleop 실행과 import 전 YAML 선택
├── il.py                         # IL command dispatcher
└── ffw_sh5_grasp/
    ├── application/              # 앱 상태, target 좌표, control loop
    ├── cli/                      # IL command별 argument parser
    ├── control/                  # 팔, 손, base, whole-body controller
    ├── imitation/                # data/simulation/act/runtime/apps/visualization
    ├── kinematics/               # tree, task, constraint, solver, collision
    └── visualization/            # teleop ImGui와 MuJoCo renderer
```

## 공개 경로

- 기구학은 `ffw_sh5_grasp.kinematics` 또는 그 하위 모듈에서 import한다.
- IL 데이터는 `ffw_sh5_grasp.imitation.data` 아래에서 import한다.
- MuJoCo policy 환경은 `ffw_sh5_grasp.imitation.simulation`에 있다.
- checkpoint 추론과 평가는 `ffw_sh5_grasp.imitation.runtime`에 있다.
- interactive IL 앱은 `ffw_sh5_grasp.imitation.apps`에 있다.

이전의 `src/ik.py`, `src/kinematics.py` 및 `imitation/dataset.py` 같은 flat wrapper는
제공하지 않는다. 호출부를 정식 모듈로 고치지 않고 호환 파일을 새로 만들지 않는다.

### 제거된 경로와 현재 경로

| 제거된 경로·API | 현재 사용법 |
|---|---|
| `src/ik.py`, `InverseKinematics` | `kinematics.JointSpaceKinematics` 또는 `KinematicTree` |
| `src/kinematics.py` | `from ffw_sh5_grasp import kinematics` |
| `imitation.dataset` | `imitation.data.episode` |
| `imitation.mujoco_env` | `imitation.simulation.environment` |
| `imitation.policy_runner` | `imitation.runtime.runner` |
| `imitation.record_app`, `policy_app` | `imitation.apps.recording`, `apps.policy` |
| `BaseTeleop.update(...)` | `update_body(...)` 후 `SwerveDrive.update_twist(...)` |
| ACT YAML `transformer_layers` | `encoder_layers`와 `decoder_layers`를 각각 지정 |

이 표의 제거된 이름은 import compatibility를 보장하지 않는다. 저장된 ACT v2
checkpoint와 ALOHA HDF5 episode schema는 코드 경로와 무관하므로 계속 사용할 수 있다.

## 모델 자산

`models/full_scene.xml`에는 현재 task가 사용하는 동적 물체만 둔다. can-to-box task의
동적 물체는 `can_free` 하나이며 목표 상자 `target_bin`은 고정 body다. 사용하지 않는
free joint는 화면에서 숨기는 방식으로 보존하지 않는다. 보이지 않아도 계속 적분되어
state 차원을 늘리고 수치 문제를 만들기 때문이다.

## 실행 파일과 생성 파일

```bash
python3 src/teleop_app.py
python3 src/il.py --help
python3 src/il.py train --config config/imitation/act.yaml
```

`imgui.ini`, `MUJOCO_LOG.TXT`, `__pycache__`, 학습 `outputs/`, dataset과 Rerun 파일은
소스가 아니다. `.gitignore` 대상으로 유지하고 `src/` 아래에 커밋하지 않는다.

## 삭제 판단 기준

다음 중 하나에 해당하면 코드를 보존하지 않고 사용처를 정식 API로 전환한 뒤 삭제한다.

- 내부 사용처가 없는 wrapper, alias 또는 사용되지 않는 인자
- 이름과 실제 동작이 달라 오해를 만드는 API
- UI에 표시되지만 제어 결과에 사용되지 않는 설정
- 숨김 처리만 된 이전 MJCF body, joint 또는 geom
- 실행 시 생성되는 로그·창 배치·캐시 파일

삭제 전에는 `rg`로 코드·테스트·문서 사용처를 확인한다. 삭제 후에는 관련 Phase,
`test_il_*.py`, Whole-body 회귀와 실제 checkpoint smoke test를 실행한다.
