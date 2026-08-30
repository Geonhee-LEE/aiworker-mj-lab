# Q-space parallel coordinates 시각화 + CVD-safe 팔레트 검증

- **Cycle**: 2026-08-30 19:00 KST
- **Branch**: (사람 주도, main 직접 작업)
- **TODO**: (신규 TODO 없음 — 사용자 직접 요청)
- **Phase**: P1 (데모 도구 개선)
- **Status**: keep

## What I tried

사용자가 "Q space를 시각화해도 좋지 않을까요"라고 제안해서, 실제 계획 결과
(오른팔 장애물 장면, 59 iteration, 시작 트리 28노드 + 목표 트리 27노드,
20-waypoint 경로)를 JSON으로 내보내 parallel coordinates 인터랙티브 페이지를
만들어 Artifact로 게시했다. `dataviz`/`artifact-design` 스킬을 먼저 로드하고
절차를 따랐다.

색을 고르는 과정에서 `dataviz` 스킬의 팔레트 검증기(`validate_palette.js`)를
돌리려 했는데 이 환경에 Node.js가 없어서, 검증기의 OKLab/CVD 시뮬레이션
수식(Machado-Oliveira-Fernandes 2009)을 Python으로 그대로 포팅해 직접
돌렸다. 처음 쓰려던 초록·파랑·주황 조합이 protanopia 시뮬레이션에서
초록-주황 Delta E 2.8(문턱 6, 목표 8)로 하드 FAIL이 나서, 여러 후보를
체계적으로 탐색해 초록(#2a9e4a)·파랑(#4f8ff2)·마젠타(#d94fa0) 조합으로
바꿨다 — 라이트·다크 모드 둘 다, CVD·명도·채도·대비 검사를 전부 통과한다.

이 색 조합을 3D MuJoCo 뷰어의 트리·경로 시각화(`demo_plan_right_arm.py`)에도
그대로 반영해서 두 시각화가 같은 색 언어를 쓰게 맞췄다.

## What worked / what failed

Node.js가 없는 환경에서 팔레트 검증기를 못 돌리는 건 예상 못 한 제약이었다.
JS 코드를 그대로 Python으로 옮겨 쓰는 건 번거로웠지만, 결과적으로 실제
CVD 실패를 잡아냈다는 점에서 "그냥 눈대중으로 괜찮아 보이는 색 고르기"보다
훨씬 나은 결과를 얻었다 — 이 스킬의 핵심 규율("색은 계산하는 것, 눈대중이
아니다")이 정확히 의도한 상황이었다.

## North-star delta

없음 — 시각화/디버깅 도구.

## Key learnings

- 색각 이상 검사는 직관과 어긋난다: 초록-주황처럼 "확실히 달라 보이는" 조합도
  protanopia에서는 거의 같은 색으로 보일 수 있다. 실제로 계산해 보기 전에는
  "그럴듯해 보인다"는 판단을 신뢰하면 안 된다.
- Q-space 자체(7차원)는 사람이 볼 수 없지만, parallel coordinates로 모든
  차원을 동시에 투영하면 "이 관절은 넓게 탐색됐고 저 관절은 좁게 움직였다"
  같은 정보가 3D Cartesian 투영에서는 안 보이던 방식으로 드러난다. 두
  시각화(3D 궤적, Q-space PCP)는 서로 다른 것을 보여주므로 상호보완적이다.
- 이 환경에는 Node.js가 없다 — 앞으로 이 스킬을 다시 쓸 때는 처음부터
  Python 포팅 경로를 고려하거나, Chrome 브라우저 자동화로 모듈 스크립트를
  돌리는 경로를 준비해 두는 게 낫다.

## Recommended next 1–3 priorities

1. MP-0005 shortcut 평활화
2. MP-0006 시간 파라미터화
3. (선택) Q-space 익스포트를 정식 스크립트(`scripts/export_qspace_viz.py`
   같은)로 승격할지 검토 — 지금은 일회성 결과물

## Artifacts

- PR: 없음(사람 직접 작업)
- Files touched: scripts/demo_plan_right_arm.py (팔레트만),
  docs/guide/motion-planning.md
- 게시물: Q-space 시각화 Artifact(비공개, 사용자 세션에 링크 전달)
- TSV row appended: no
