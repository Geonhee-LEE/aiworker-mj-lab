# 인터랙티브 마커가 바닥 밑에 렌더링되던 버그 수정

- **Cycle**: 2026-08-30 22:02 KST
- **Branch**: (사람 주도, main 직접 작업)
- **TODO**: (신규 TODO 없음 — 사용자가 "노랑 구슬이 땅바닥 아래에 있다"고 보고)
- **Phase**: P1 (데모 도구 버그 수정)
- **Status**: keep

## What I tried

`--interactive`로 띄운 노란 구슬(`goal_marker`)이 항상 바닥 밑에 보인다는
사용자 보고를 재현했다. `data.mocap_pos[marker_id] = initial_state.position`로
목표 위치를 쓴 뒤 `mj_forward`를 호출하지 않았다는 걸 코드로 직접 확인했다 —
`mocap_pos`를 쓰는 것만으로는 렌더링에 실제 쓰이는 `data.xpos`가 갱신되지
않는다. mocap body의 world pose는 `mj_kinematics`/`mj_forward`가 다시
돌아야 `mocap_pos`에서 재계산된다. 그래서 구슬은 컴파일 시점 기본값(월드
원점, `pos`를 따로 안 줬으므로 `[0,0,0]`)에 그대로 남아 있었고, 바닥
(`floor` geom, `z=0.148`)보다 낮아 "땅 밑에 있는 구슬"처럼 보였다.

## What worked / what failed

먼저 순수 `model`/`data` 스크립트로 `body xpos`가 `mj_forward` 전에는
`[0,0,0]`이고 후에는 정확한 값으로 바뀜을 재현했다. 그다음 실제
`main(["--interactive"])` 경로에 임시 후킹을 넣어 뷰어가 뜬 상태에서
`data.xpos[marker_body]`를 직접 읽어, 수정 전엔 원점 근처, 수정 후엔
바닥보다 위(z≈0.99 vs 바닥 z=0.148)임을 확인했다.

## North-star delta

없음 — 데모 버그 수정.

## Key learnings

- MuJoCo에서 `mocap_pos`/`mocap_quat`를 프로그램적으로 쓴 뒤에는, 그
  값이 렌더링에 반영되는 `data.xpos`/`data.geom_xpos`로 전파되도록
  `mj_forward`(또는 최소 `mj_kinematics`)를 명시적으로 한 번 더 불러야
  한다. 사용자가 뷰어 안에서 직접 드래그할 때는 뷰어 자체의 내부 렌더
  루프가 알아서 갱신하지만, **파이썬 코드가 최초로 mocap 위치를 설정하는
  순간에는** 그 자동 갱신을 기대할 수 없다.
- 이 종류의 버그는 "값은 맞는데 화면에 안 보인다"는 증상으로 나타나서,
  로그 출력(`print(data.mocap_pos[...])`)만 보면 정상으로 보인다 — 반드시
  렌더링에 실제 쓰이는 `data.xpos`까지 확인해야 잡을 수 있다.

## Recommended next 1–3 priorities

(자율 루프의 `STATE.md` 우선순위를 그대로 따른다 — MP-0006 시간 파라미터화,
MP-0013 벤치마크 하네스)

## Artifacts

- PR: 없음(사람 직접 작업, main 직접 커밋)
- Files touched: scripts/demo_plan_right_arm.py
- TSV row appended: no
