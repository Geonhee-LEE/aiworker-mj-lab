# 인터랙티브 IK에서 자세(orientation) 목표 제거 — 위치만 푼다

- **Cycle**: 2026-08-30 22:16 KST
- **Branch**: (사람 주도, main 직접 작업)
- **TODO**: (신규 TODO 없음 — 사용자가 IK 미수렴을 반복 보고하며 "모션
  플래닝이 이 문제를 풀어주는 게 아니었나" 질문)
- **Phase**: P1 (데모 도구 버그 수정) — P4 IK 설계에도 참고할 발견
- **Status**: keep

## What I tried

사용자가 인터랙티브 모드에서 여러 지점을 드래그했을 때 "IK가 수렴하지
않았습니다"가 반복된다고 보고했다. 보고된 실패 지점(`[-0.403, -0.865, 1.419]`,
`[-0.157, -0.753, 1.389]`)을 그대로 재현해서, 세션 시작 시점의 손 자세를
고정한 채(`hold_quat`, `kinematics.tasks.pose_error`로 위치+자세를 동시에
DLS로 풂) IK를 25번 재시도해도 첫 번째 지점은 전혀 안 풀리는 걸 확인했다.
같은 지점에 대해 **자세 제약 없이 위치 3개 자유도만** DLS로 풀면 즉시(첫
후보에서) 수렴하고 충돌도 없는 해가 나왔다. `_ik_attempt`/`_solve_valid_ik`를
position-only로 다시 짰다 — `kinematics.tasks.pose_error`와 `hold_quat`을
아예 뺐다.

## What worked / what failed

세 지점(사용자 보고 2개 + 이전에 82 iteration 걸렸던 지점) 모두 position-only
버전에서 즉시 수렴 + 충돌 없는 해를 찾았다. 실제 `main(["--interactive"])`
경로로 세 지점을 순서대로 드래그하는 시뮬레이션을 돌려 전부 계획 성공까지
확인했다.

## North-star delta

없음 — 데모 IK 버그 수정. 다만 이 발견은 P4(Cartesian goal, MP-0011)를 설계할
때도 그대로 적용된다: **자세를 반드시 명시해야 하는 게 아니라면 위치만
제약하는 게 IK 성공률을 훨씬 높인다.**

## Key learnings

- **IK 수렴 실패와 모션 플래닝은 서로 다른 층의 문제다.** 모션 플래닝
  (RRT-Connect)은 "이미 존재하는 두 관절 configuration(시작·목표) 사이에
  충돌 없는 경로가 있는가"를 푼다. IK는 그 이전 단계로 "이 3D 점(+자세)을
  만족하는 관절 configuration이 애초에 존재하는가"를 푼다. IK가 실패하면
  목표 configuration 자체가 없다는 뜻이라 플래닝이 개입할 여지가 없다 —
  경로를 찾을 대상(목표점)이 없기 때문이다.
- 이번 실패의 원인은 "그 위치가 물리적으로 도달 불가능"이 아니라 **불필요한
  과잉 제약**(고정된 자세)이었다. 7-DOF 팔에게 위치(3) + 자세(3) = 6개
  제약을 동시에 걸면 남는 여유 자유도는 1개뿐이라 특정 자세를 고집하면
  기구학적으로 막히는 영역이 꽤 넓어진다. 사용자 입력이 실제로 표현하는
  정보(3D 점 하나)보다 많은 제약을 임의로 추가하면 안 된다.

## Recommended next 1–3 priorities

(자율 루프의 STATE.md 우선순위를 그대로 따른다 — MP-0006 시간 파라미터화,
MP-0013 벤치마크 하네스). 추가로: P4에서 `planning.goals`를 설계할 때 이번
position-only 발견을 반영할 것.

## Artifacts

- PR: 없음(사람 직접 작업, main 직접 커밋)
- Files touched: scripts/demo_plan_right_arm.py, docs/guide/motion-planning.md
- TSV row appended: no
