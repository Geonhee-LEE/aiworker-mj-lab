# 릴리스 기록

## 3.1.0 — Research Report, Grad-CAM & Cleanup

2026-08-25 발행. [GitHub Release](https://github.com/ggh-png/aiworker-mj-lab/releases/tag/v3.1.0)

### 보고서와 분석

- `IK_Teleoperation_v3.pdf`의 연구 흐름을 실제 코드·manifest·rollout 로그와 교차검증한
  [ACT 연구 보고서](research-report.md) 추가
- 보고서의 수집/학습 주기, D97/D150 색상 개수와 PTE 범위를 현재 구현 기준으로 정정
- action-target Grad-CAM CLI와 lossless NPZ를 추가하고, Joint/Task의 공정한 비교를 위한
  signed world-frame EE-Y 폐루프 분석 방법과 인과 해석 한계를 문서화
- PTE 성공률·penalized mean time 및 데이터/성공·실패 분포 그래프 생성기 추가

### 코드 구조

- Joint/Task 학습의 중복 `modular_dataset_loader`, `modular_trainer`,
  `modular_training_config`를 공용 dataset/trainer/config와 representation strategy로 통합
- `train-modular`는 단일 `train` 구현으로 연결하고 오래된 standalone entrypoint와 policy
  wrapper를 제거
- checkpoint representation, dataset statistics, split과 policy catalog 회귀를 보강

### 배포와 검증

- 공개 Hugging Face
  [데이터셋](https://huggingface.co/datasets/ggh-png/ffw-sh5-can-color-sort)과
  [모델](https://huggingface.co/ggh-png/ffw-sh5-act-color-sort)의 카드·manifest·사용법 갱신
- 두 Hub 저장소에 코드 릴리즈와 대응하는 `v3.1.0` revision tag 추가
- Ruff, pytest 44개와 `mkdocs build --strict` 통과

전체 diff: [v3.0.0...v3.1.0](https://github.com/ggh-png/aiworker-mj-lab/compare/v3.0.0...v3.1.0)

## 3.0.0 — Modular ACT Color Sorting

2026-08-23 발행. [GitHub Release](https://github.com/ggh-png/aiworker-mj-lab/releases/tag/v3.0.0)

### 환경과 데이터

- 초록·빨강·주황·파랑 캔과 좌우 상자 색 배치를 reset마다 독립 무작위화
- 상자 wall/floor contact, 오른팔 작업 공간과 물리 기반 성공 판정 정리
- 150개 성공 HDF5 episode에 58,676 frame, 양팔 상태/action, 양쪽 EE pose와 세 RGB 저장
- 공개 [Hugging Face 데이터셋](https://huggingface.co/datasets/ggh-png/ffw-sh5-can-color-sort) 배포

### 학습과 추론

- 오른팔 Joint/Task 8D 표현을 같은 ACT 구조·seed·split 조건으로 학습
- Task-space 출력의 quaternion 정규화 및 bounded differential IK 실행
- UI/CLI에서 representation과 PTE 미래 step을 선택하고 Rerun 기록 주기 제어
- D97/D150 × Joint/Task × PTE 0/5/10/15/20을 조건당 100회, 총 2,000 rollout 평가
- 네 best checkpoint와 평가 CSV를 공개
  [Hugging Face 모델 저장소](https://huggingface.co/ggh-png/ffw-sh5-act-color-sort)에 배포

### 품질과 문서

- Ruff/pytest CI, strict MkDocs build/deploy와 용도별 requirements 추가
- Hub release manifest와 SHA-256 검증, 공개 다운로드·재배포 절차 문서화
- PTE/Rerun 종료 회귀와 modular Joint/Task 학습 테스트 보강

전체 diff: [v2.1.0...v3.0.0](https://github.com/ggh-png/aiworker-mj-lab/compare/v2.1.0...v3.0.0)

## 2.1.0 — Repository & IL Workflow Refresh

2026-08-20 발행. [GitHub Release](https://github.com/ggh-png/aiworker-mj-lab/releases/tag/v2.1.0)

- 저장소 이름을 `aiworker-mj-lab`으로 확장하고 기존 Python package import 유지
- arm-only HDF5 record/replay, ACT train/evaluate와 Rerun command 경계 정리
- GitHub Pages의 사용자·개발자 문서 구조 갱신

전체 diff: [v2.0.0...v2.1.0](https://github.com/ggh-png/aiworker-mj-lab/compare/v2.0.0...v2.1.0)

## 2.0.0 — Explicit Whole-body QP & Safety Projection

2026-08-07 발행. [GitHub Release](https://github.com/ggh-png/aiworker-mj-lab/releases/tag/v2.0.0)

### 제어와 수치 해법

- 18-DOF 전신 differential IK에 명시적 box-QP active-set 해법 추가
- 자유도별 damping, base 참여율과 joint-limit CBF를 velocity bound 문제에 통합
- geometry 최근접점 기반 self/table collision gradient와 별도 soft-barrier safety
  projection 적용
- Bimanual rigid-grasp, world-fixed target, 수동 주행 handover와 실제 스워브 추종 안정화

### 화면과 문서

- target/current 손 pose와 오차 시계열을 비교하는 Pose Graph 탭 추가
- Pseudoinverse, DLS, QP의 공통 task와 서로 다른 수치 경로 문서화
- Whole-body 명목 solve, base shaping, collision projection의 실제 호출 순서 반영

### 검증

- 작은 box-QP의 완전탐색 optimum 비교
- joint-limit/collision CBF, arm-only/base participation hard gate 검증
- Phase 0–6, Whole-body 통합과 `mkdocs build --strict` 통과

전체 diff: [1.4.0...v2.0.0](https://github.com/ggh-png/aiworker-mj-lab/compare/1.4.0...v2.0.0)

## 1.4.0 — Lean Runtime & Faster IK Defaults

2026-08-03 발행. [GitHub Release](https://github.com/ggh-png/aiworker-mj-lab/releases/tag/1.4.0)

### 코드와 API

- RPY·quaternion·회전행렬 계산을 `kinematics.rotations`에 통합
- `TeleopApp`의 단순 전달 wrapper 16개와 Whole-body private 호환 별칭 제거
- 좌우 팔 controller/목표 상태를 `arm_controllers[side]`, `q_des[side]`로 통합
- 스워브 피드백, 손가락 관절 범위, marker 표시와 UI jog의 중복 분기 정리
- 소스 코드 순 90줄 이상 축소

### 제어와 설정

- 단일 팔 DLS의 관절 step과 반복 상한을 높여 먼 목표 수렴 여유 확대
- Whole-body 위치·자세 gain과 말단/base/lift/팔 속도 상한을 높여 목표 추종 가속
- 모든 기본값과 검증 규칙은 한국어 주석이 포함된 `config/default.yaml`에서 관리

### 문서와 검증

- 제거·이동된 함수와 실제 직접 호출 구조를 API 및 모듈 가이드에 반영
- Phase 0–6, YAML 설정, Whole-body 통합 테스트 통과
- Python compile, diff whitespace와 `mkdocs build --strict` 통과

전체 diff: [1.3.0...1.4.0](https://github.com/ggh-png/aiworker-mj-lab/compare/1.3.0...1.4.0)

## 1.3.0 — Modular Kinematics & Unified Developer Guide

2026-08-02 발행. [GitHub Release](https://github.com/ggh-png/aiworker-mj-lab/releases/tag/1.3.0)

### 구조와 코드

- `kinematics/solver.py`를 공개 facade와 단일 팔 solver에 집중시키고, 회전 수학,
  MJCF tree/FK/Jacobian, collision distance gradient를 독립 모듈로 분리
- `control/whole_body.py`에서 BVLS/soft barrier와 양손 rigid-grasp 순수 계산을 분리
- `KinematicTree`를 양손 solver가 공유해 모델 topology 중복 해석 제거
- 기존 `kinematics` 공개 import와 `InverseKinematics` 호환 경로 유지
- 사용하지 않는 wrapper·helper와 중복 ImGui 런타임 상태 파일 정리
- 핵심 제어 흐름과 복잡한 수치 처리에 한국어 주석 보강

### 문서와 학습 경로

- 시스템 이해, 개발자 가이드와 ROS2 관점을 하나의 내비게이션으로 통합
- Tree → FK/Jacobian → Quaternion → Collision → DLS → IK → WBIK 순서로 재구성
- 기구학 tree를 만드는 이유부터 FK/Jacobian과 collision gradient까지 수식·코드·테스트 연결
- 상황별 공개 함수, 반환값과 다음 호출을 찾는 API 함수 지도 추가
- 기반 지식은 기존 상세 문서 링크로 연결하고 구현된 수학의 완주 범위 명시
- 모방학습 episode/step schema, replay, shadow mode, sim-to-real 안전 절차 가이드 추가
- Whole-body/Arm-only 데모 영상을 올바른 YouTube 링크로 교체

### 검증

- Phase 0–6 전체 통과
- Whole-Body 통합 테스트 통과
- FK/Jacobian 중앙 유한차분 최대 오차 `2.33e-10`
- collision distance gradient 중앙 유한차분 최대 오차 `7.39e-11`
- 단일 팔 무작위 IK 100/100 수렴
- 무작위 WBIK 40/40 descent·read-only·bound 통과
- `mkdocs build --strict`와 diff whitespace 검사 통과

전체 diff: [1.2.0...1.3.0](https://github.com/ggh-png/aiworker-mj-lab/compare/1.2.0...1.3.0)

## 1.2.0 — Custom Kinematics & Compact Multi-Viewport UI

2026-07-23 발행. [GitHub Release](https://github.com/ggh-png/aiworker-mj-lab/releases/tag/1.2.0)

### 사용자에게 보이는 변화

- ImGui 도구가 메인 MuJoCo 창 밖의 실제 네이티브 OS 창으로 분리
- 6개 도구 창을 `Control Center`와 `Diagnostics` 두 워크스페이스로 통합
- Target, 양팔 IK/FK, Robot/Grasp, Kinematic Tree, Joint Monitor를 탭으로 정리
- multi-viewport desktop 좌표를 반영해 3D gizmo 중심을 선택 target에 정확히 정렬
- 수식 전개, 기능별 알고리즘→코드 대응, 제어 다이어그램을 개발자 문서에 확장

### 기구학과 제어

- 컴파일된 MJCF의 body·joint·site 고정 변환을 `KinematicTree`로 한 번 복사
- hinge/slide FK와 world-aligned 6×N geometric Jacobian을 NumPy로 직접 계산
- collision 최근접점의 translational Jacobian도 같은 트리에서 직접 계산
- 단일 팔 DLS IK와 18-DOF Whole-Body IK가 하나의 custom tree를 공유
- 앱 런타임에서 `mj_forward`, MuJoCo Jacobian API, `site_xpos/site_xmat` FK 우회 제거
- MuJoCo는 `mj_step` 기반 접촉·동역학·actuator·렌더링에만 사용

### 검증

- 런타임 custom-kinematics dependency gate 통과
- FK/Jacobian 중앙 유한차분 최대 오차 `2.33e-10`
- collision distance gradient 중앙 유한차분 최대 오차 `7.39e-11`
- 단일 팔 무작위 IK 100/100 수렴, pick 10/10
- compact UI gate: workspace 2개, 실제 플랫폼 viewport는 주 창 포함 3개
- Phase 0–6, Whole-Body 통합, strict 문서 빌드 통과

전체 diff: [1.1.1...1.2.0](https://github.com/ggh-png/aiworker-mj-lab/compare/1.1.1...1.2.0)

## 1.1.1 — Whole-body Control Toggle

2026-07-19 발행. [GitHub Release](https://github.com/ggh-png/aiworker-mj-lab/releases/tag/1.1.1)

### 사용자에게 보이는 변화

- `Lift / Utilities`에 **Whole-body Control ON/OFF** 버튼 추가
- 상태줄에 `ON` 또는 `OFF (arm-only)`와 실제 body command 표시
- 전환 시 양손/virtual-object target의 world pose 보존
- OFF에서도 keyboard base와 manual lift 사용 가능
- `V`/checkbox collision CBF 시각화와 완화된 3 cm/1 cm 기준

### 제어와 알고리즘

- OFF에서 base x/y/yaw와 lift differential velocity hard pin
- 공용 ROS-free pose/FK/Jacobian 계층
- bounded whole-body BVLS와 joint-limit CBF
- Bimanual rigid-grasp relative-pose task
- arm-arm/body/table signed-distance collision CBF
- 키 해제 제동과 manual-to-WBIK rebase 안정화

### 검증

- Phase 0–6 전체 통과
- 일반/Bimanual ON→OFF→ON pose 불변성
- arm-only base/lift zero + 양팔 error descent
- collision gradient/CBF와 visualization consistency
- 무작위 WBIK 40회와 실제 바퀴 4방향 추종
- strict 문서 빌드

전체 diff: [1.1.0...1.1.1](https://github.com/ggh-png/aiworker-mj-lab/compare/1.1.0...1.1.1)

## 1.1.0 — ROS-free Whole-body IK

- base 3축 + lift + 양팔 14축 differential IK
- 실제 steer/drive actuator와 wheel-ground contact 기반 mobile control
- 키보드 해제 후 잔류 wheel command와 원점 복귀 방지

## 이전 버전

`1.0.0`, `0.1.0`, `0.0.2`, `0.0.1`과 `phase-0`~`phase-4` 태그가 있다. 최신
사용법과 검증 기준은 항상 현재 문서를 우선한다.
