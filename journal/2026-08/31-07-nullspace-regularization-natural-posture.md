# 인터랙티브 IK에 nullspace 정칙화 추가 — 자연스러운 자세 유도

- **Cycle**: 2026-08-31 07:17 KST
- **Branch**: (사람 주도, main 직접 작업)
- **TODO**: (신규 TODO 없음 — 사용자가 "목적지까지는 가지만 팔 모양이
  부자연스럽다, 매니퓰레이터 모션 플래닝을 보통 이렇게 푸는지" 질문)
- **Phase**: P1 (데모 도구 개선) — P4 IK 설계에도 참고할 발견
- **Status**: keep

## What I tried

position-only IK(직전 cycle)는 위치 3개만 제약하고 나머지 4개 여유
자유도를 완전히 무작위(어느 무작위 재시도가 먼저 수렴하는지)에 맡겼다 —
그래서 매번 자세가 크게 바뀌었다. 표준적인 여유 매니퓰레이터 redundancy
resolution 공식(`dq = J⁺e + (I − J⁺J)(k(q_ref − q))`)을 `_ik_attempt`에
추가했다: 주 목표(위치)의 nullspace 안에서만 "현재 관절값에 가깝게
유지"라는 부목표를 민다. 이 저장소의 반응형 `WholeBodyIK`가 이미 쓰는
`regularization_task`와 같은 발상이다.

## What worked / what failed

nullspace 이득(gain)을 0~0.5로 스윕해서 (a) 수렴 실패를 안 만드는지
(b) 실제로 현재 자세에서 덜 벗어나는지 확인했다. 0.5는 두 지점에서
수렴 자체를 깨서 0.2로 정했다. 사용자가 "부자연스럽다"고 지적한 지점
중 하나(`[-0.157, -0.753, 1.389]`)를 직접 분석해 보니, **현재 자세에서
가장 가까운 해가 실제로 IK로는 수렴하지만 장애물과 충돌**해서, 무작위
재시도가 완전히 다른(더 멀리 떨어진) 유효한 자세를 찾은 것이었다 —
즉 그 지점의 "부자연스러움"은 버그가 아니라 장애물을 피하려면 실제로
크게 재배치해야 한다는 물리적 사실이었다. nullspace 정칙화는 이런
경우엔 여전히 멀리 떨어진 해를 낸다(의도된 동작 — 충돌 회피가 정칙화
보다 우선).

세 지점 모두 실제 `main(["--interactive"])` 경로로 재검증했고, RRT-Connect
반복 횟수도 줄었다(예: 24→5) — 시작·목표가 관절 공간에서 더 가까워졌기
때문으로 보인다.

## North-star delta

없음 — 데모 IK 품질 개선. P4(`planning.goals`) 설계에 nullspace
정칙화를 기본으로 반영할 근거가 됐다.

## Key learnings

- **"부자연스러워 보이는 해"를 볼 때 먼저 확인할 것: 그게 정말 자유도를
  낭비한 결과인지, 아니면 장애물이 실제로 그 재배치를 강제한 것인지.**
  둘을 구분 안 하면 정칙화로 "고치려다" 충돌 회피를 망가뜨릴 수 있다.
- 여유 매니퓰레이터 IK에서 redundancy resolution(nullspace 정칙화)은
  선택 사항이 아니라 사실상 필수다 — 안 하면 해마다 자세가 임의로
  달라져서 사람이 보기에 "로봇이 이상하게 움직인다"는 인상을 준다.
- hydrax(github.com/vincekurtz/hydrax)는 GPU 기반 sampling MPC(MPPI,
  CEM, DIAL-MPC 등) 라이브러리이지 RRT류 경로 탐색기가 아니다 — receding
  horizon 최적 제어 문제를 푼다. 이 저장소에 들여온다면 RRT-Connect를
  대체하기보다, 아직 없는 P3(정식 실행 모듈)에서 "전역 경로(RRT-Connect)를
  따라가는 매끄러운 저수준 제어"로 보완하는 역할이 자연스럽다. 이 머신에
  RTX 5080 + CUDA 12.8이 있어 기술적으로는 시도 가능하지만, JAX/MJX라는
  무거운 새 의존성이 필요하고 hydrax README는 CUDA 13을 명시한다 — 실제
  설치 검증이 먼저 필요하다. 아직 구현 착수는 안 함, 사용자 확인 대기.

## Recommended next 1–3 priorities

(자율 루프 STATE.md 우선순위를 그대로 따른다 — MP-0006 시간 파라미터화,
MP-0013 벤치마크 하네스). hydrax 통합은 사용자 확인 후 별도 TODO로 등록할
것.

## Artifacts

- PR: 없음(사람 직접 작업, main 직접 커밋)
- Files touched: scripts/demo_plan_right_arm.py, docs/guide/motion-planning.md
- TSV row appended: no
