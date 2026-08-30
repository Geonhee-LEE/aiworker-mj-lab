# 탐색 영역에 장애물 추가 + 실행 중 경로 시각화 유지

- **Cycle**: 2026-08-30 17:30 KST
- **Branch**: (사람 주도, main 직접 작업)
- **TODO**: (신규 TODO 없음 — 사용자 직접 요청, 데모 스크립트 확장)
- **Phase**: P1 (데모 도구 개선)
- **Status**: keep

## What I tried

1. **장애물 추가**: `mujoco.MjSpec.from_file` + `spec.worldbody.add_geom(...)` +
   `spec.compile()`로 저장소의 `models/full_scene.xml`을 전혀 건드리지 않고
   데모 실행 시점에만 빨간 기둥(`planning_obstacle`, 테이블 위 x=0.4055
   y=0.0 z=0.95, 반지름 5×5×20cm)을 붙였다. 별도 contype/conaffinity 없이
   저장소 기본값(1/1)을 물려받아 자동으로 오른팔과 충돌한다.
2. **경로 시각화 유지**: `--execute` 재생을 시작하기 직전에 최종 경로를
   주황색으로 그리고, 재생이 끝날 때까지 지우지 않는다. `_execute`의 프레임
   콜백이 `user_scn`을 건드리지 않으므로 자연스럽게 유지된다.

## What worked / what failed

처음 기둥 크기(5×5×15cm)로는 무작위 유효 목표 30개 중 하나도 직선 경로가
막히지 않았다 — 완전 무작위 샘플링이 작은 국소 장애물을 우연히 지나칠
확률이 낮기 때문이다. 크기를 5×5×20cm로 키우고 "테이블 근처로 필터링한"
후보 39개 중 9개(~23%)가 막히는 것을 확인한 뒤 이 크기로 확정했다.
`checker.report(q).pair_name`에 `planning_obstacle`이 실제로 등장하는
configuration을 찾아 장애물이 진짜로 충돌 판정에 관여함을 직접 검증했다.

경로 시각화는 트리 시각화와 자료구조가 같다는 걸 이용했다 — 경로의 각
waypoint는 "부모가 바로 앞 waypoint인 사슬"이므로 `_draw_trees` 헬퍼를
그대로 재사용하고 새 렌더링 코드를 안 짰다.

## North-star delta

없음 — 데모 도구 개선. P1 알고리즘 자체는 변화 없음.

## Key learnings

- `mujoco.MjSpec`으로 컴파일 전 모델에 지오메트리를 추가하면, 컴파일된
  `MjModel`을 직접 mutate하는 `enable_task_collisions` 같은 기존 함수와
  아무 충돌 없이 조합할 수 있다(순서: MjSpec 편집 → compile() → 기존 함수로
  후처리).
- 무작위 목표 샘플링으로 장애물 회피를 "보여주려면" 장애물이 작업공간에서
  차지하는 비율이 충분히 커야 한다 — 작은 장애물은 시각적으로는 있어도
  데모에서 거의 안 걸린다.

## Recommended next 1–3 priorities

1. MP-0005 shortcut 평활화
2. MP-0006 시간 파라미터화
3. (선택) 장애물 회피가 "항상" 보이는 curated start/goal 쌍을 기본값으로
   제공할지 검토 — 지금은 확률적으로만 보여준다

## Artifacts

- PR: 없음(사람 직접 작업)
- Files touched: scripts/demo_plan_right_arm.py, docs/guide/motion-planning.md
- TSV row appended: no
