# Research State — auto-generated each cycle

_Last updated: 2026-09-07 · cycle p0-todo-tool-unit-tests_

## North star distance

**P0 인프라 gap 하나를 닫았다.** `scripts/todo_tool.py`(모든 cycle이 매
사이클 의존하는 TODO.md 파싱/렌더링/CLI 도구)에 지금까지 테스트가 전혀
없었다 — `tests/test_todo_tool.py` 17개 신규(파싱·렌더링 왕복,
`_next_id`, `_escape`/`_unescape`, CLI `add`/`set`/`check`/`next`를
격리된 임시 `TODO_PATH`로 검증)로 이 gap을 메웠다(PR #16). 부수적으로
`parse()`가 `str.split("|")`로 표 셀을 나누는데 `render()`의 백슬래시
이스케이프(`\|`)를 이해하지 못해 제목에 리터럴 `|`가 있으면 행이 조용히
버려지는 기존 버그를 발견 — 이번엔 characterization test로만 문서화하고
수정은 후속 TODO(P0, 신규 등록)로 미뤘다.

이 전 cycle에서는 safety-certificate 스타일 캐싱 도입 여부를
프로파일링으로 판단해 "도입하지 않는다"는 실측 근거 있는 결론을 냈다
(PR #15, MP-0028 완료). 그 전에는 P4 Cartesian pose goal IK(PR #14,
MP-0011)와 RRT-Connect vs RRT* 50-seed 비교(PR #13, MP-0007/MP-0017)가
완료됐다 — 셋 다 아직 사람 리뷰 대기.

## Current bottleneck

**PR 리뷰 큐가 4건으로 늘어 다음 cycle cadence gate(≥4)에 걸린다.**
#13(MP-0007/MP-0017 벤치마크 비교)·#14(MP-0011 Cartesian pose goal
IK)·#15(MP-0028 safety-certificate 프로파일링)·#16(MP-0019 todo_tool.py
단위 테스트) 전부 사람 리뷰 대기 — 다음 executor cycle은 조용히
`EXECUTOR_SKIP reason=pr-queue-full`로 종료될 가능성이 높다. `MP-0012`
(offline_pose_ik.py 위임)는 PR #14가 병합돼 `planning/goals.py`가
main에 들어와야 착수 가능.

## Open experiments

| Branch | Last update | Last description | Days open |
|---|---|---|---|
| planning/p0-todo-tool-unit-tests | 2026-09-07 | PR #16, MP-0019 todo_tool.py 단위 테스트 17개 — 사람 리뷰 대기 | 0 |
| planning/p5-planner-comparison | 2026-09-06 | PR #13, MP-0007/MP-0017 실측 비교 — 사람 리뷰 대기 | 1 |
| planning/p4-cartesian-pose-goal-ik-seed | 2026-09-06 | PR #14, MP-0011 Cartesian pose goal IK 다중 재시도 — 사람 리뷰 대기 | 1 |
| planning/p1-safety-certificate-profiling | 2026-09-06 | PR #15, MP-0028 safety-certificate 캐싱 순이득 프로파일링(도입 보류) — 사람 리뷰 대기 | 1 |

## Recent learnings (last 3 cycles)

- **파싱 코드에 이스케이프 로직이 있으면 그 역방향(파서)도 실제로
  이스케이프를 이해하는지 왕복(round-trip) 테스트로 확인해야 한다.**
  `todo_tool.py`의 `_escape`/`render`는 올바르게 백슬래시를 붙이지만
  `parse`의 naive `split("|")`는 그걸 몰라 셀 개수가 어긋나 행이 조용히
  사라진다 — 코드 리뷰만으로는 놓치기 쉬운 비대칭이다.
- **"테스트만 작성" 스코프에서 버그를 발견해도 바로 고치지 않고
  characterization test로 문서화 + 후속 TODO 등록이 맞는 절차** — 한
  cycle/브랜치당 작업 방향 하나 유지 원칙과 일치한다.
- **TODO.md는 feature 브랜치 커밋에 포함하지 않는다** — 과거 병합 PR
  전부 확인 결과 canonical한 상태 변경은 오직 main의 `state_push.sh`를
  거친다. Phase 3에서 `todo_tool.py set --status Doing`으로 생긴 로컬
  diff는 커밋 전에 되돌려야 한다(이번 cycle에서 실제로 걸려 고쳤다).
- **"바로 구현하지 않고 프로파일링부터"로 스코프를 좁힌 research TODO는
  "하지 않는다"는 결론도 유효한 성과다.** MP-0028: 프로파일링 스크립트와
  실측 데이터(TSV)를 남겨두면 장면/모델이 바뀌었을 때 같은 도구로
  재평가할 수 있다.

## Next claude-actionable

1. **신규 TODO(등록 예정)**: `todo_tool.py` `parse()`를 이스케이프-aware
   셀 분리기로 교체 — 제목에 `|`가 있으면 행이 조용히 버려지는 버그 수정.
2. `MP-0022`(P4) — aggregate_results.py에 Wilson CI 계산 추가(MP-0018은
   이미 Done으로 확인됨, 바로 착수 가능).
3. `MP-0012`(P4) — PR #14가 병합돼 `planning/goals.py`가 main에 들어오면
   바로 착수 가능(현재는 실행가능성 필터에 걸려 건너뜀).

## Next user-blocked

1. **PR #13/#14/#15/#16** 사람 리뷰/병합 대기 — 4건으로 큐 포화 임박.
2. **`MP-0030`** P7 Tier 2(결합형 whole-body 플래너) 착수 여부 결정 —
   타당성 평가 완료, 검증 시나리오(합성 MJCF)가 먼저 필요하다는 점까지
   문서화됨.
3. **`MP-0020`** Telegram 봇 생성 및 `telegram_setup.sh` 실행 (사람만
   가능).
4. **`MP-0021`**(hydrax)/`MP-0025`(VAMP-MR) 우선순위 결정 — 둘 다 조사는
   끝났고 구현 여부만 사용자 판단 대기.
5. **`MP-0014`**(pose goal 20 seed 성공률 측정) — `UserTest=☑`, 사람 확인
   필요.

## Cycles to date

20 (2026-08-30~09-06 사람 주도: P0 부트스트랩, P1 RRT-Connect 구현, 데모
반복/트리 시각화, 장애물 재배치, Q-space 시각화+CVD 팔레트, 인터랙티브 마우스
목표+버그 수정 3건, nullspace 정칙화+hydrax 조사, 데모 실행 경로에 shortcut+
시간 파라미터화 연결, RRT* 대안 플래너, CHOMP류 궤적 최적화 후처리, 벤치마크
하네스+P1 성공률 첫 측정, PR #1/#2 병합+P3 실행 모듈, PRD를 P7(모바일
매니퓰레이터)까지 확장 + P7.0 reachability map, P7.1 베이스 자세 선택,
P7 Tier 2 타당성 평가 + PR #3/#4/#5/#7/#8/#12 전체 병합(문서 중복 수동
정리 포함)로 PR 큐 완전 소진; 자율 루프: shortcut 평활화, 시간
파라미터화, MP-0007/MP-0017 벤치마크 비교, P4 Cartesian pose goal IK
다중 재시도, safety-certificate 캐싱 순이득 프로파일링(도입 보류),
todo_tool.py 단위 테스트)
