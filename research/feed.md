# Research feed

_cap 30, 최신이 위. REVIEW 단계는 상위 5개만 읽는다._

- [2026-09-24] [024](2026-09/024.md) 새 각도: CPU 다중 코어 활용은
  001~023에서 한 번도 안 다뤄짐 — Ichnowski IROS12의 lock-free 단일
  트리 병렬화는 R-NF-005(결정론) 위반 위험+대수술이라 기각, 대신
  "parallel restarts"(서로 다른 파생 시드로 독립 RRT* 시도 K개를
  `multiprocessing`으로 동시 실행해 최선 채택)를 제안 — 하위 시드를
  `seed*1000+i`로 명시 파생하면 결정론 유지 가능. MP-0031의
  narrow_passage/cluttered 78-88% 성공률은 표본 실패에 가까워 다중
  시도로 완화될 여지가 있고, MP-0032/0039/0044(단일 시도 개선)와
  직교해 곱해질 수 있는 레버. 착수 전 `mujoco.MjModel` pickling
  가능 여부 확인이 먼저 필요(미검증, executor 몫). RRT-Connect는
  이미 100%+12.5ms라 프로세스 풀 오버헤드가 이득을 상쇄해 제외. 신규
  TODO 1건(MP-0048, P5, owner=claude). PR 큐 여전히 4건(#18~#21)
  11일째 포화(재조회 생략, curator 로그 참고).

- [2026-09-23] [023](2026-09/023.md) MP-0033(LSPB via-point 블렌딩)의
  설계 근거를 재확인하다가 한계 발견 — 선형 속도 블렌딩은 waypoint
  완전정지는 없애지만 가속도(jerk) 연속성은 보장하지 않아 정지 불연속을
  블렌딩 구간 경계로 옮길 뿐일 수 있음(가설, 미실측). jerk-limited
  7-segment S-curve(Biagiotti-Melchiorri 교과서, Ruckig OTG
  arXiv:2105.04830)는 closed-form이라 LP/외부 solver 없이 numpy로 구현
  가능 — TOPP-RA(feed 010에서 LP 의존으로 배제)보다 §7 원칙에 더 부합.
  LSPB 대체가 아니라 실측 시 승격 가능한 2차 후보로 backlog 등록. 부수
  확인: `reachability.py`가 역방향(per-voxel IK) 방식인데 문헌은 순방향
  FK 샘플링이 수 배 더 싸다고 보고 — 단, R-F-011은 "1회 구축·재사용"
  전제라 이미 병합된 구현을 바꿀 근거 부족, 조치 없음(negative finding).
  신규 TODO 1건(MP-0047, owner=claude, P2). PR 큐 여전히 4건
  (#18/#19/#20/#21) 10일째 포화(재조회는 생략, curator 로그 참고).

- [2026-09-22] [022](2026-09/022.md) 새 각도: `_Tree.nearest`/`near`(RRT-Connect·RRT* 공통)가
  노드마다 `np.asarray`+`norm`을 부르는 순수 파이썬 선형 탐색(~1.35µs/노드)인데
  docs가 "충분히 빠르다"고만 서술할 뿐 측정 기록이 저장소에 없음. 격리 마이크로벤치로
  ndarray+einsum 벡터화 시 50~100× 확인. `p5-planner-comparison.tsv`의 RRT*가 15s
  예산에 iterations 3065~3084로 거의 상수(반복당 ~4.9ms)라 시간이 NN 스캔(트리 크기
  비례)에 지배될 가능성 — 맞다면 MP-0031의 RRT* 성공률 78-88%와 MP-0032/0039/0044의
  A/B 기준선이 반복 수 부족에 오염돼 있을 수 있음(추정, cProfile 선행 필요).
  RRT-Connect는 트리가 작아 이득 미미해 제외. 신규 TODO 1건(MP-0046, P5, owner=claude).
  PR 큐 4건(#18~#21) 10일째 포화.

- [2026-09-21] [021](2026-09/021.md) PRD §1에서 유일하게 "미측정"인 실행 안전성
  행(재생 중 최소 clearance ≥ 0.01m)을 코드와 대조 — `clearance()`는 있으나 경로
  위에서 호출하는 곳이 없고 TODO도 없음(MP-0041은 이론 상한 테스트라 별개). 계획+
  shortcut 경로를 0.005rad로 재표집해 최솟값을 기록하면 MP-0041 위험의 경험적 확인이
  된다. 부수 점검: MuJoCo 이슈 #2710(`mj_geomDistance`가 distmax 대신 DBL_MAX 반환)은
  `collision_distance_gradient`가 이미 두 동작 모두 걸러내 조치 불필요(코드 실독,
  로컬 mujoco 미설치라 버전 재확인은 못 함). 신규 TODO 1건(MP-0045, P3, owner=claude).
  PR 큐 4건(#18~#21) 9일째 포화.

- [2026-09-20] [020](2026-09/020.md) 새 각도: RRT-Connect/`RightArmSpace.sample`이
  지금까지 순수 균등 무작위(`rng.uniform`)만 쓴다는 걸 확인 — 저불일치
  (low-discrepancy) 시퀀스(Halton/Sobol)로 바꾸면 표본 수 적은 초반 트리
  확장에서 커버리지가 나아진다는 최근 문헌(Halton BIT* 2026, Enhanced
  RRT*+Halton 2025)이 있으나, raw Halton은 7차원에서 기수(2~17)
  상관관계로 오히려 역효과 날 수 있어 scrambling이 필수라는 함정도 확인.
  R-NF-005(`np.random.default_rng`만 사용) 제약과는 스크램블 순열을
  `rng.permutation()`으로 파생시키면 상충 안 함(설계 스케치만, 미검증).
  신규 TODO 1건(MP-0044, owner=claude, P5, MP-0032보다 낮은 우선순위로
  명시). PR 큐 여전히 4건(#18/#19/#20/#21) 8일째 포화, backlog 착수
  우선 권고 유지.
- [2026-09-19] [019](2026-09/019.md) 새 각도: redundant 7-DOF 팔의 Cartesian
  pose 목표는 IK 해가 여러 개인데, `solve_pose_goal_multistart`(P4)는
  `q_seed` 실패 후 무작위 재시도에서 "첫 수렴 해"를 검증 없이 그대로
  RRT-Connect `q_goal`로 넘긴다 — q_init에서 관절공간상 더 가까운(플래너가
  더 쉬운) 해가 나머지 미시도 후보 중에 있어도 평가하지 않음. OMPL
  `GoalStates`/Many-RRT*(arXiv:2603.04547) 문헌은 이걸 조기 확정 문제로
  본다. 멀티트리 재설계(Many-RRT*)는 이 저장소의 "RRT*는 단일 트리" 결정과
  충돌해 제외, 대신 "최대 M개 수렴 해를 모아 q_seed 최근접 선택"하는
  축소판을 제안 — 플래너 무수정, 기존 함수 재사용. can-sort는 장애물이
  적어 신호가 안 보일 수 있어 narrow_passage(MP-0031 기존 시나리오)로
  A/B 권장. 신규 TODO 1건(MP-0043, owner=claude, P4). PR 큐 여전히 4건
  포화(#18/#19/#20/#21), backlog 착수 우선 권고 유지.
- [2026-09-18] [018](2026-09/018.md) 신규 각도 둘 확인. (1) GJK/narrow-phase
  심플렉스 워밍스타트(연속 configuration 간 temporal coherence로 근접
  충돌쿼리 가속) — 실제 mujoco 3.12.0 파이썬 바인딩 점검 결과
  `mj_geomDistance`/`mj_collision` 모두 심플렉스 상태를 노출하지 않아
  엔진 패치 없이는 적용 불가, 막다른 길로 기록. (2) MJX(MuJoCo XLA) 기반
  배치 병렬 충돌검사 — GMT*/VAMP/pRRTC/Kino-PAX(2025~) 문헌은 GPU 배치
  sampling으로 고차원 계획을 가속, OMPL 비의존이라 PRD Non-Goal은 피하지만
  JAX/CUDA 전이 의존 + 대규모 재작성이 필요해 hydrax/VAMP-MR과 같은
  owner=user 버킷으로 신규 TODO 1건(MP-0042). PR 큐 여전히 4건 포화,
  017의 "backlog 착수 우선" 권고 재확인.
- [2026-09-17] [017](2026-09/017.md) 후보 주제(CCD 완결성/RRT-Connect
  step_size_rad 튜닝/CHOMP obstacle cost)를 각각 재확인했지만 셋 다 신규
  조치로 이어지지 않았다 — CCD는 016(MP-0041)이 이미 커버, step_size_rad는
  문헌 유효범위(15°~135°) 안이고 MP-0004/0017 실측(100% 성공, 12.5ms)이
  튜닝 유인 자체를 안 주며, CHOMP obstacle cost는 R-F-004b가 의도적으로
  제외한 설계이고 겹치는 후보(MP-0040 partial shortcut)가 이미 있음.
  신규 TODO 0건 — 부정적 결과 기록, 다음 cycle은 backlog 착수 우선 권장.
- [2026-09-16] [016](2026-09/016.md) `EdgeChecker`의 고정 `resolution_rad`
  (0.05, 매직 넘버 리터럴)가 `padding_m`(0.012m)과 아무 유도 관계 없이
  독립적으로 정해졌음을 확인 — CCD/conservative advancement 문헌
  (arXiv:1909.10857)의 완결성 조건("스텝 변위 상한 < clearance 하한")에
  대입하면 관절에서 ~24cm 이상 떨어진 원위 링크에서는 변위가 padding을
  넘어설 수 있어, 이론적으로 두 인접 표본 사이에 얇은 장애물을 놓칠 수
  있음(관측된 버그는 아님 — 현재 장면은 장애물이 두꺼워 미발현). 새 CCD
  구현 대신 안전 상한 검증 단위테스트부터 추가하는 최소안 제시. 신규
  TODO 1건(MP-0041, owner=claude, P1).
- [2026-09-15] [015](2026-09/015.md) PRD가 미결로 남긴 질문(P2 exit
  criterion 30%↓ vs 실측 중앙값 6.82%)에 대한 근본 원인 후보 발견 —
  Partial Shortcut(Geraerts & Overmars 2007)은 whole-vector shortcut이
  "한 DOF만 막혀도 나머지 여유 DOF까지 전부 컷 실패"하는 구조적 한계를
  겨냥한 대안으로, DOF 하나씩만 보간·검사한다. 장애물이 적은(구체 3개)
  can-sort 장면일수록 이 실패모드가 두드러질 수 있어 6.82% 정체를
  설명할 후보. 단, 원 논문도 "매니퓰레이터 환경은 유일한 예외일 수
  있다"고 경고 — 직접 A/B 필요. 신규 TODO 1건(MP-0040, owner=claude, P2).
- [2026-09-14] [014](2026-09/014.md) `rrt_star._sample`의 `use_informed`
  거부표집(`max_rejections=20`, 실패 시 균등 폴백)을 원 논문(Gammell et al.,
  Informed RRT*, IROS 2014)과 대조 — 그 논문이 "고차원/큰 세계에서 거부표집은
  성공확률이 임의로 작아진다"며 정확히 경고한 실패 모드를 이 저장소가 7-DOF
  관절공간에서 재현 중일 가능성. 해가 최적에 근접할수록(informed subset이
  좁아질수록) `max_rejections`가 소진돼 수렴 후반부에 조용히 균등표집으로
  되돌아감 — informed 효과가 가장 필요한 순간 비활성화. 수정안은 numpy SVD만
  으로 되는 direct ellipsoid sampling(외부 의존 없음, §7 원칙 부합). 신규
  TODO 1건(MP-0039, owner=claude, P5).
- [2026-09-13] [013](2026-09/013.md) 신규 주제 대신 기존 backlog 두 건을
  착수 가능한 수준으로 구체화. (1) MP-0032 bridge test 정확한 알고리즘
  확인 — 두 invalid 점의 중점이 valid일 때만 채택하는 고전 Hsu et al.
  방식, 오목 모서리 거짓양성 약점, MP-0031 기존 narrow_passage/cluttered
  시나리오로 A/B 가능. (2) TOTG(Kunz-Stilman, RSS 2012)를 R-F-010의
  MP-0033(LSPB) 다음 2차 후보로 기록 — LP/spline 의존 없이 위상평면
  수치적분+원호 blend로 코너 정지 문제를 경로 자체를 고쳐 해결, 전역
  시간 최적. PR 큐 포화(#17/#18/#19/#20, 여전히 4건) 변동 없음. 신규
  TODO 0건(둘 다 기존 항목 커버).
- [2026-09-12] [012](2026-09/012.md) 후보 주제 목록(RRT류/충돌가속/MuJoCo
  API)이 반복 조사로 소진돼, PRD에 "계획, 미착수"로 남은 R-F-009(IK 목표
  탐색 개선)를 이번 cycle 주제로 확인. `demo_plan_right_arm.py`의
  `_solve_valid_ik`가 PRD 지적 그대로 관절 전체 범위 균등 무작위 재시도만
  쓰는 걸 코드로 재확인, `q_init` 우선 시도는 이미 warm-start 문헌 권장과
  일치해 조치 불필요. 문헌은 "무작위 대신 reachability map/인접 격자
  시드가 계산 낭비를 줄인다"고 일관되게 보고 — `ReachabilityMap`(R-F-011)이
  성공률 스칼라만 저장하고 실제 `q`는 버리는 것도 확인. 1차(q_init 반경
  확장 샘플링, 즉시 착수)/2차(representative_q 저장 확장, R-F-011 스키마
  변경) 두 단계로 분리해 신규 TODO 2건(MP-0037 owner=claude, MP-0038
  owner=user). PR 큐는 여전히 4건·6일째 그대로(`gh pr list` 재확인).
- [2026-09-11] [011](2026-09/011.md) BIT*(Batch Informed Trees) 문헌 확인 —
  고차원 매니퓰레이터에서 RRT 계열보다 빨리 수렴한다고 보고되나, P5 실측
  (MP-0007/MP-0017)이 이미 "RRT*조차 RRT-Connect 대비 경로 품질 이득
  통계적으로 없음"으로 나온 이 저장소 규모에서는 즉시 도입 근거가 약함 —
  hydrax/VAMP-MR과 같은 취급으로 owner=user TODO만 등록(MP-0036), 구현은
  로드맵 확장 결정 대기. MuJoCo `contype`/`conaffinity` broad-phase
  필터링 재확인 결과 `ArmCollisionChecker`의 기존 설계(모델의 `<exclude>`
  상속)가 이미 문헌 권장 패턴과 일치 — feed 002의 "프로파일링 먼저"
  결론 재확인, 신규 조치 없음. 신규 TODO 1건.
- [2026-09-10] [010](2026-09/010.md) PR 큐 포화로 executor가 못 움직이는
  동안 기존 방향 두 가지를 문헌으로 재확인 — (1) `toppra`는 numpy/scipy+LP
  solver 의존이라 "무의존 순수 파이썬"이 아님을 확인, MP-0033(LSPB via-point
  blending)을 R-F-010 1차 후보로 둔 feed 008 판단이 맞았음을 재확인.
  (2) Lazy edge collision checking 문헌(Hauser ICRA 2015) 확인 결과
  `EdgeChecker`의 기존 bisection 조기기각은 이미 그 아이디어의 일부이고,
  RRT* rewiring 단계의 진짜 lazy evaluation은 실측 병목이 확인되기 전이라
  TODO화하지 않음. 신규 TODO 0건(기존 MP-0033이 이미 커버).
- [2026-09-09] [009](2026-09/009.md) `ArmCollisionChecker.clearance()`가
  호출하는 `collision_distance_gradient`(3개 모드 전부)가 호출부가 쓰지도
  않는 Jacobian을 쌍마다 2회씩 계산하는 낭비를 코드 실독으로 확인 —
  `is_valid()`는 이미 gradient-free라 문제없음. `need_gradient=False` 조기
  반환 분기 하나로 고칠 수 있고, MP-0028의 safety-certificate 순이득
  판단(현재 리뷰 대기 PR #15)이 이 낭비 포함 기준선 위에서 나온 결론일
  가능성도 제기(재프로파일링 필요, 가설). 신규 TODO 1건(MP-0035).
- [2026-09-08] [008](2026-09/008.md) R-F-010(연속 동작 매끄러움) 미착수
  문제의 근본 원인을 `planning/trajectory.py` 모듈 docstring에서 확인 —
  현재 사다리꼴 프로파일은 세그먼트 경계에서 항상 완전 정지한다(방향 전환 시
  가속도 발산 버그를 피하려 의도적으로 그렇게 설계됨, MP-0006 범위 밖으로
  명시). **LSPB via-point blending**(코너 근처에서 이전/다음 세그먼트 속도를
  선형 결합, 완전 정지 없이 통과)을 hydrax(MP-0021, GPU 의존성 승인 대기)
  없이 순수 파이썬으로 `time_parameterize`에 opt-in 추가 가능한 1차 후보로
  제안. TOPP-RA/SAAC는 이론적으로 더 우수하지만 spline+LP 구현 비용이 커
  2차 후보로 보류. 신규 TODO 1건(MP-0033) — 이 항목을 추가하다가
  `todo_tool.py`의 기존 버그(제목에 파이프 문자가 있으면 행이 조용히
  버려짐, MP-0033으로 이미 등록돼 있었음)가 그 버그를 설명하는 항목 자체를
  실제로 삼켜버리는 걸 실측으로 재현 — 유실된 항목을 MP-0034로 복구.
- [2026-09-07] [007](2026-09/007.md) MP-0031 실측(narrow_passage/cluttered에서
  RRT* 성공률 78-88%, RRT-Connect는 100%)의 원인을 좁은 통로 균등 샘플링의
  고전적 실패 모드로 확인 — **bridge test**(Hsu et al. 2003, 무효점 2개의
  중점을 후보로) 샘플링을 `rrt_star._sample`에 일정 확률로 섞으면 새
  라이브러리 없이 성공률을 개선할 여지가 있음. GVP-RRT(2024)는 유사 환경에서
  11.5~69.5%p 개선 보고. Bidirectional 결합형(FRRT*-Connect)은 이 저장소의
  "RRT*는 의도적으로 단일 트리" 설계 결정과 충돌해 스코프 밖으로 제외. 신규
  TODO 1건(MP-0032, 먼저 프로파일링 후 `--obstacle-layout`으로 성공률 실측).
- [2026-09-06] [006](2026-09/006.md) PR 큐 완전 소진 후 다음 착수 후보인
  MP-0017(RRT-Connect vs RRT* 50-seed 비교표)을 위해 통계 검정 방법을 미리
  확정 — 동일 seed로 두 플래너를 돌리는 대응 표본 구조이므로 독립 표본
  검정(Mann-Whitney) 대신 **Wilcoxon signed-rank test**(대응 t-test의
  비모수 버전) + 중앙값 effect size를 함께 보고할 것, 한쪽만 실패한 seed는
  성공 교집합에서 제외하고 교집합이 너무 작으면 검정 자체를 생략. MP-0022
  (Wilson CI)는 성공률 축이라 이것과 별개. 신규 TODO 0건(MP-0017이 이미
  커버).
- [2026-09-05] [005](2026-09/005.md) 충돌 검사 가속 후보로 safety-certificate
  스타일 캐싱(Bialkowski et al., IJRR 2016) 조사 — 한 번 정밀 검사한 점의
  `clearance()` 반경 안에서는 이후 `is_valid`를 생략할 수 있다는 아이디어.
  다만 이 저장소는 인증서 반경을 얻으려면 `is_valid`보다 훨씬 비싼
  `clearance()`를 호출해야 해 순이득이 반경≫`resolution_rad`일 때만 나온다 —
  실측 없이 판단 불가. 바로 구현하지 않고 프로파일링부터 하는 걸로 스코프를
  좁힌 신규 TODO 1건(MP-0028, 벤치마크 하네스 PR #7 병합 후 자연스러움).
- [2026-09-04] [004](2026-09/004.md) 병목이 PR 리뷰 큐라 신규 설계보다 다음
  착수 후보(MP-0008 실행 feedforward, MP-0011 IK 시딩) 검증에 집중. 문헌
  확인 결과 둘 다 기존 계획(velocity feedforward는 보류 후 필요시 추가,
  IK는 이전 성공 해+reachability 격자 시딩)이 이미 올바른 방향임을 재확인
  — 특이점 근처에서는 warm-start만으론 부족해 무작위 재시도 폴백 병행이
  필요하다는 근거(GNN warm-start 100%→93% 사례)만 추가. 신규 TODO 0건.
- [2026-09-03] [003](2026-09/003-vamp-mr.md) 사용자 제안 VAMP-MR(SIMD 가속
  multi-arm 플래너) 검토 — GitHub API로 사실관계 확인 결과 기술 주장은
  정확(10-100x, IROS 2026, Baxter dual-arm 지원 등). 다만 PRD Non-Goal(외부
  플래닝 라이브러리, OMPL 명시 제외)과 정면 충돌 + `libompl-dev` 등 전이
  의존성으로 실제로 OMPL을 끌고 들어옴 + sudo 시스템 패키지가 필요한 무거운
  C++ 빌드. hydrax(MP-0021)와 같은 취급으로 owner=user TODO만 등록,
  구현은 사용자의 명시적 PRD 수정 + 시스템 설치 승인 대기. 신규 TODO 1건
  (MP-0025).
- [2026-09-02] [002](2026-09/002.md) MP-0013 성공률은 raw 퍼센트 대신 Wilson
  score 95% CI로 보고할 것 — n=50 근처에서 반폭이 ~±8~10%p라 "45/50=90%"가
  실제로는 임계값 미달일 수 있음. `collision_state.py._forward`가 매 `is_valid`
  마다 스크래치 모델 전체(양팔·베이스 등)에 `mj_kinematics`+`mj_collision`을
  돌리는 구조 확인 — 2분 예산 초과 시 `state_checks` 카운터로 먼저 프로파일링
  후 `<exclude>` 보강, 모델 축소는 실측 없이 하지 말 것. 신규 TODO 1건
  (MP-0022, aggregate_results.py에 CI 계산 추가).
- [2026-09-01] [001](2026-09/001.md) MP-0013 벤치마크 하네스: 주 지표는 예산 내
  성공률, TSV엔 seed별 raw 행(집계는 별도)을 남겨 나중에 percentile/CDF 재계산
  가능하게 할 것, baseline vs shortcut/time_parameterize는 변수 하나만 바꿔
  비교(factorial). MP-0008 실행 모듈: `ArmTorqueController.apply`는 목표 속도가
  없어 궤적 추종 중 항상 위상 지연이 남을 수 있음(velocity feedforward 옵션
  기록) — 다만 P3 성공 기준이 "최종 정지 오차"라 먼저 feedforward 없이 측정 후
  필요시 추가 권장. 신규 TODO 없음(MP-0013/MP-0008/MP-0009가 이미 커버).
- [2026-08-31] [001](2026-08/001.md) MP-0006 시간 파라미터화: 관절별 사다리꼴
  시간을 계산해 `max_j(t_j)`로 세그먼트 동기화 + 역산 스케일 권장(moveit
  IterativeParabolicTimeParameterization과 동일 계열). TOPP-RA는 더 짧은
  실행시간을 낼 수 있으나 외부 라이브러리 의존 우려로 지금은 보류, P5 비교
  연구 후보로만 남김.
