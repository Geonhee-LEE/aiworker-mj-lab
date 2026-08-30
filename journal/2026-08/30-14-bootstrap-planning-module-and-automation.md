# 오른팔 모션 플래닝 부트스트랩 + 자동 연구 루프 배선

- **Cycle**: 2026-08-30 14:00 KST
- **Branch**: (사람 주도, main 직접 작업)
- **TODO**: `MP-0001` [planner] `RightArmSpace` + `ArmCollisionChecker` + `EdgeChecker` (P0 기반)
- **Phase**: P0
- **Status**: keep

## What I tried

`~/Representation-Aware-MPPI`의 cron + Telegram 자동 연구 루프 패턴을 조사해
이 저장소에 맞게 이식했다(Notion 대신 `TODO.md` 파일 단독 권위, auto-merge 없음,
Telegram 발송기에 4096자 청킹 추가). 동시에 오른팔 7-DOF sampling-based 모션
플래닝 모듈(`src/ffw_sh5_grasp/planning/`)의 P0 기반을 설계·구현했다.

## What worked / what failed

기존 `kinematics/collision.py`의 `default_collision_pairs`는 상자(`target_bin*`)
geom을 포함하지 않고, 상자 자체가 raw MJCF에서 `contype=2 conaffinity=0`이라
오른팔과 충돌하지 않는다는 것을 확인했다. 플래너가 이를 모르면 상자를 그냥
관통하는 "안전한" 경로를 반환할 위험이 있어, `ArmCollisionChecker` 생성자에
`require_contact_geoms` 가시성 가드를 필수로 넣었다.

## North-star delta

P0 완료로 "충돌 여부를 안전하게 판정할 수 있다"는 기반이 생겼다. 아직 실제
경로를 만들지는 못한다(P1 대기).

## Key learnings

- scratch `MjData`는 반드시 `copy.deepcopy(model)` 위에서 만들어야 한다 —
  `imitation.simulation.environment`가 상자 충돌을 런타임에 승격시키므로
  raw XML 재파싱은 이 승격을 놓친다.
- `can_free`(7-qpos free joint)를 스냅샷에서 빼먹으면 캔이 원점으로
  순간이동한 채로 충돌 판정을 하게 된다.

## Recommended next 1–3 priorities

1. MP-0002 RRT-Connect 코어
2. MP-0003 무충돌 경로 속성 시험
3. MP-0018 `aggregate_results.py` (벤치마크보다 먼저 준비)

## Artifacts

- PR: 없음(초기 부트스트랩, main 직접 커밋)
- Files touched: docs/prd.md, docs/todo.md, docs/agents.md, docs/skills.md,
  docs/automation.md, TODO.md, STATE.md, JOURNAL.md, RESULTS.md,
  src/ffw_sh5_grasp/planning/*, scripts/*
- TSV row appended: no (P0에는 정량 지표 없음, P1부터 시작)
