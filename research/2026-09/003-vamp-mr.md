# 003 — VAMP-MR(SIMD 가속 multi-arm 플래너) 통합 검토

날짜: 2026-09-03
관련 TODO: MP-0025(신규, owner=user)
관련 Phase: P5(대안 플래너 비교) 인접, 통합 시 P0~P3 전반에 영향

## 배경

사용자가 Kavraki Lab의 `VAMP-MR`(SIMD 가속 multi-arm 샘플링 플래너)을 이
저장소(왼팔+오른팔 dual-arm)에 도입할 수 있는지 제안했다. GitHub API로
`KavrakiLab/vamp`와 `vamp-mr/vamp-mr` 저장소 메타데이터·README를 직접
확인해 사실관계를 검증했다(가짜/과장 여부 확인 목적) — 결론: **기술 주장은
정확했다.** 다만 이 저장소에 지금 들여오기엔 구조적 장벽이 세 가지 있어
`hydrax`(MP-0021)와 같은 방식으로 조사만 하고 owner=user TODO로만 등록했다.

## 1. 확인된 사실

- **`KavrakiLab/vamp`**: Apache 2.0, C++, ICRA 2024 논문("Motions in
  Microseconds via Vectorized Sampling-Based Planning"). 구체(sphere) 근사
  기하에 AVX2/NEON SIMD로 FK+충돌검사를 벡터화 — Franka Panda 기준 중앙값
  35 마이크로초. Baxter(양팔)도 이미 지원 로봇 목록에 있다(dual-arm 벤치마크
  시나리오 `bookshelf_tall_both_arms_*` 포함).
- **`vamp-mr/vamp-mr`**: 별도 조직·저장소. Philip Huang·Chenrui Gao·
  Jiaoyang Li(CMU/Michigan), **IROS 2026 채택**. VAMP의 단일 로봇 충돌검사를
  "여러 로봇 간 상대 변환·부착물·장애물을 함께 다루는" 범용 multi-arm
  충돌검사 모듈로 확장. 계획·후처리·실행에서 10~100배 가속을 보고 —
  사용자가 인용한 수치와 정확히 일치.
- 사용자가 말한 "GPU 불필요, 순수 CPU, ROS 불필요"도 정확하다.

## 2. 이 저장소에 지금 들여오면 안 되는 이유

1. **PRD Non-Goal과 정면 충돌.** `docs/prd.md` §1 Non-Goals: "OMPL/cuRobo 등
   외부 플래닝 라이브러리 도입 — 저장소 원칙상 직접 구현". `vamp-mr`는 빌드
   의존성에 `libompl-dev`가 **직접** 들어 있다 — PRD가 명시적으로 배제한
   바로 그 라이브러리를 전이적으로 끌고 들어온다.
2. **"기존 FK에 바로 연결된다"는 정확하지 않다.** VAMP(-MR)의 속도는 자체
   컴파일된 구체 기반 SIMD FK/충돌 커널에서 나온다 — `kinematics/tree.py`의
   기존 FK를 재사용해서 얻는 게 아니다. 실제로 쓰려면 (a) `cricket` 툴로
   FFW-SH5 팔 URDF/메시를 구체로 근사하는 별도 자산 파이프라인을 새로
   만들어야 하고, (b) 구체 전용 충돌 모델이 `ArmCollisionChecker`가 이미
   인코딩한 안전 계약(손 내부 접촉 제외, 테이블/상자 margin, 상자 가시성
   승격, CBF buffer와의 padding 관계 등)을 그대로 보존하는지 별도 검증이
   필요하다 — 단순 배선 작업이 아니다.
3. **무거운 네이티브 빌드 = 시스템 변경.** `vamp-mr`는 CMake, Ninja, Boost,
   OMPL, Protobuf, TBB, yaml-cpp, Eigen3를 `sudo apt-get install`로 깔고
   `sudo cmake --install`로 `/usr/local`에 설치해야 한다. 이 저장소는 지금
   순수 Python(+MuJoCo)이고 executor 하드 리밋(`docs/agents.md`)이 이미
   "crontab/systemctl/apt/pip install로 시스템 변경 금지"를 명시한다 — 자동
   루프는 물론 사람이 수동으로 들여올 때도 상당한 결정이다. 게다가
   IROS 2026 채택 직후의 23-star 저장소로, 특정 데모(dual-arm LEGO 조립)
   중심의 이른 단계 연구 코드다.

## 결론 / 다음 단계

`hydrax`(MP-0021)와 같은 취급 — 조사만 하고 `MP-0025`로 `owner=user` TODO에
등록했다. 사용자가 (1) PRD Non-Goal을 명시적으로 수정하고 (2) sudo 시스템
패키지 설치를 승인하면 그때 실제 통합 설계에 착수한다. 그 전까지 executor
풀에서 구조적으로 제외된다(owner=user).

만약 나중에 착수한다면 최소 스코프는: `cricket`으로 오른팔(우선)의 구체
근사 생성 → `ArmCollisionChecker`와 나란히 두고 같은 seed 집합에서 유효성
판정이 일치하는지 교차 검증하는 property test → 그 다음에야 계획 성능
비교. 왼팔까지 포함한 진짜 dual-arm 통합은 그 다음 단계다.

## 출처

- https://github.com/KavrakiLab/vamp (README, GitHub API 메타데이터)
- https://github.com/vamp-mr/vamp-mr (README, GitHub API 메타데이터)
- https://openreview.net/pdf?id=ePPOoz8KKp (VAMP-MR 논문)
