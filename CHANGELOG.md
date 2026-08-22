# Changelog

## 3.0.0 — 2026-08-23

- 초록·빨강·주황·파랑 캔과 좌우 목표 상자 배치를 reset마다 독립적으로 무작위화하는
  색상 분류 환경 및 물리 contact 기반 성공 판정 추가
- 양팔 joint/qvel/action, 양쪽 EE pose와 `cam_high`·양쪽 wrist RGB를 함께 저장하는
  HDF5 schema 1.1 수집·검증·replay 경로 완성
- 동일한 ACT 하이퍼파라미터로 오른팔 8D joint-space와 task-space 정책을 학습하고,
  task 출력은 bounded differential IK로 실행하는 modular 학습 경로 추가
- 기존 ACT temporal ensemble과 미래 offset을 사용하는 PTE를 UI/CLI에서 전환하고,
  97/150 episode × Joint/Task × PTE 0/5/10/15/20의 2,000 rollout 평가 지원
- Rerun 이미지 기록 주기·압축을 제어 loop와 분리하고 Viewer 연결 종료가 정책 실행을
  중단하지 않도록 종료 경로 보강
- 데이터셋과 네 정책을 공개 Hugging Face Hub 저장소로 검증·배포하는 manifest 및
  SHA-256 기반 release 도구 추가
- Ruff/pytest GitHub Actions, 재현 가능한 docs/Hugging Face/presentation 의존성 파일과
  strict MkDocs 배포 workflow 추가

## 2.1.0 — 2026-08-20

- 저장소 이름을 `aiworker-mj-lab`으로 확장하고 Python 패키지 호환성을 유지
- IL command dispatcher와 arm-only ACT record/train/evaluate/Rerun 파이프라인 정리
- 사용자·개발자 문서와 GitHub Pages 배포 경로 갱신

## 2.0.0 — 2026-08-07

- 18-DOF whole-body differential IK를 명시적 box-QP active-set 해법으로 확장하고,
  자유도별 damping·base 참여율·joint-limit CBF를 같은 제약 문제에 통합
- geometry 최근접점 기반 self/table collision distance gradient와 soft-barrier CBF
  safety projection, collision 진단 시각화 추가
- 양손 rigid-grasp, world-fixed target, 수동 주행 handover와 실제 swerve 물리 추종을
  안정화하고 관련 회귀를 보강
- Control Center에 target/current 손 pose 및 오차 시계열을 비교하는 Pose Graph 탭 추가
- Whole-body IK 수식 증명, 코드 아키텍처/API 지도, Forward Kinematics 발표용 인포그래픽을
  포함해 사용자·개발자 문서를 전면 갱신

## 1.4.0 — 2026-08-03

- 회전·쿼터니언 변환을 `kinematics.rotations`의 단일 구현으로 통합
- `TeleopApp`의 좌표·렌더 전달용 wrapper와 사용하지 않는 private 호환 별칭 제거
- 좌우 팔 controller와 목표 관절각을 `side` 기반 딕셔너리로 통합
- 스워브 피드백, 파지 관절 범위, UI jog와 marker 표시의 중복 분기 정리
- 단일 팔 DLS 및 Whole-body IK의 gain·속도 기본값을 빠른 목표 추종 기준으로 조정
- 실제 공개 함수와 호출 구조에 맞춰 API 및 개발 문서 갱신
- Phase 0–6, YAML 설정, Whole-body 통합 및 엄격 문서 빌드 검증

## 1.3.0 — 2026-08-02

- 런타임을 application/control/kinematics/visualization 계층으로 분리
- 모든 조절값을 한국어 주석이 포함된 YAML 설정으로 중앙화
- 함수별 API 지도와 통합 개발자 학습 경로, 모방학습·sim-to-real 가이드 추가
- Whole-body/Arm-only 데모 링크와 알고리즘 상세 구현 연결 보강

## 1.2.0 — 2026-07-23

- 컴파일된 MJCF에서 body–joint–site 트리를 만드는 `KinematicTree`와 자체 FK 구현
- hinge/slide site Jacobian과 충돌점 Jacobian을 직접 계산하고 런타임의
  `mj_forward`/`mj_jacSite`/`mj_jac` 및 engine site-pose 우회 제거
- 단일 팔 DLS IK와 18-DOF Whole-Body IK가 동일한 custom kinematics 계층 공유
- ImGui 네이티브 multi-viewport로 도구를 실제 OS 창으로 분리하고 gizmo 좌표 정렬
- 6개 도구 창을 `Control Center`와 `Diagnostics` 두 tabbed workspace로 통합
- FK·Jacobian·DLS·collision CBF 수식 전개와 코드 대응, 다이어그램 중심 문서 재구성
- custom kinematics 의존성, 수치 Jacobian, UI workspace를 포함한 회귀 gate 강화

## 1.1.1 — 2026-07-19

- 전신 제어 ON/OFF UI와 무점프 whole-body/arm-only 전환
- OFF 상태에서 base x/y/yaw와 lift IK 속도를 정확히 0으로 고정
- 공용 FK/Jacobian 기반 ROS-free IK 개선과 Bimanual rigid-grasp 제약
- arm/상체/table reactive collision CBF, 시각화 토글, 완화된 3 cm 감시·1 cm 안전거리
- 스워브 정지/수동 handover 안정화 및 반복 headless 회귀 테스트

## 1.1.0

- ROS-free 18-DOF whole-body differential IK
- 실제 steer/drive actuator와 wheel-ground contact 기반 모바일 제어
- 키보드 해제 후 잔류 주행과 원점 복귀 방지
