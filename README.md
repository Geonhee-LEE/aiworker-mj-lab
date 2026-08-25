# aiworker-mj-lab

ROBOTIS AIWORKER(FFW-SH5)를 MuJoCo에서 연구하는 프로젝트입니다. 손의 목표 pose를
지정하면 직접 구현한 FK/Jacobian, differential IK와 controller가 명령을 계산하고,
MuJoCo actuator와 실제 contact가 다음 상태를 만듭니다. 로봇 상태를 순간 이동시켜
결과를 만드는 대신 **목표 → 기구학 → 제어 → 물리 → 관측**의 폐루프를 유지합니다.

같은 시뮬레이션 위에서 양팔·전신 텔레오퍼레이션과 ACT 모방학습을 연결합니다. 시연의
RGB·관절·EE pose·action을 같은 25 Hz tick에 기록하고, Joint/Task-space ACT와 PTE를
실제 closed-loop rollout으로 비교합니다.

[문서 사이트](https://ggh-png.github.io/aiworker-mj-lab/) ·
[빠른 시작](docs/getting-started.md) ·
[모방학습 안내](docs/guide/il/index.md) ·
[ACT 연구 개요](docs/research-report.md) ·
[v3.1.0 릴리스](https://github.com/ggh-png/aiworker-mj-lab/releases/tag/v3.1.0)

## 이 프로젝트가 다루는 것

- ROS 없이 실행되는 양팔·리프트·스워브 베이스의 Whole-body/Arm-only 제어
- 직접 구현한 FK, geometric Jacobian, differential IK와 collision CBF
- weld 없이 손가락 contact force와 마찰로 유지하는 파지
- 4색 캔 분류를 위한 HDF5 시연 데이터, Joint/Task ACT와 PTE
- 동일 seed closed-loop 평가와 action-target Grad-CAM 분석

## ACT closed-loop 비교

![D150 Joint와 Task 정책에서 PTE f=5, 10, 15, 20을 같은 초기 상태로 비교한 종합 GIF](docs/assets/evaluation/d150-joint-task-pte-f05-f20.gif)

같은 seed 195958와 초록 캔을 사용한 D150 rollout입니다. 위 행은 Joint, 아래 행은
Task이며 열은 `f=5/10/15/20`입니다. 이 GIF는 동작 차이를 보여주는 단일 예시이고,
성공률 통계와 해석은 [평가 결과](docs/evaluation-results.md)에 있습니다.

## 영상

| Whole-body Control | Arm-only Control |
|---|---|
| [![Whole-body Control 데모](https://img.youtube.com/vi/AXAByoi5CxU/hqdefault.jpg)](https://www.youtube.com/watch?v=AXAByoi5CxU) | [![Arm-only 데모](https://img.youtube.com/vi/2LV_RsAGdz8/hqdefault.jpg)](https://www.youtube.com/watch?v=2LV_RsAGdz8&list=PLWyQPsEn5Atg&index=2) |
| base·lift·양팔이 함께 목표를 추종 | base·lift를 고정하고 팔만 목표를 추종 |

## 문서와 공개 자산

설치, 실행 명령과 개발 절차는 README에 중복하지 않고 문서 사이트에서 관리합니다.

- [설치와 첫 실행](docs/getting-started.md)
- [모방학습 명령어](docs/imitation-commands.md)
- [공개 D97/D150 정책과 150-episode 데이터셋](docs/huggingface.md)
- [시스템 구조와 개발](docs/guide/index.md)
- [테스트와 검증](docs/testing.md)
