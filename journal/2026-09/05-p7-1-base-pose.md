# P7.1: 베이스 자세 선택(planning/base_pose.py)

- **Cycle**: 2026-09-05
- **Branch**: `planning/p7-1-base-pose`
- **TODO**: `MP-0027`
- **Phase**: P7
- **Status**: keep

## What I tried

PR #9(PRD 확장)·#10(P7.0 reachability map)이 병합된 뒤, 계획대로 다음
조각인 **P7.1 `planning/base_pose.py`**에 착수했다 — reachability map
소비, 베이스 발자국 충돌 검사, 기존 팔 계획기와의 end-to-end 연결.

구현 전에 `models/full_scene.xml`의 `base_link` 관절 선언부터 다시
확인했다: `base_x`/`base_y`는 world x/y축을 따르는 slide, `base_yaw`는
z축 둘레 hinge뿐이고, `base_link`의 `pos="0 0 0.15"`는 고정값이다 —
즉 베이스는 z 방향으로는 **절대** 움직이지 않는다(높이는 별도의
`lift_joint`가 담당). 이걸 확정한 뒤 이 모듈의 핵심인 SE(2) 변환
(`world_to_base_frame`)을 작성했다: `dx, dy = target - base_xy`를
`-base_yaw`만큼 회전시키고, z는 그대로 통과시킨다.

```python
def world_to_base_frame(target_world_xyz, base_pose):
    dx = target_world_xyz[0] - base_x
    dy = target_world_xyz[1] - base_y
    cos_t, sin_t = math.cos(-base_yaw), math.sin(-base_yaw)
    relative_x = cos_t * dx - sin_t * dy
    relative_y = sin_t * dx + cos_t * dy
    return np.array([relative_x, relative_y, target_world_xyz[2]])
```

`BaseFootprintChecker`는 `ArmCollisionChecker`(`planning/collision_state.py`)
의 아키텍처를 그대로 재현했다 — scratch `copy.deepcopy` 모델, live
`MjData`는 `set_snapshot`에서 한 번만 읽고, `mj_kinematics`+`mj_collision`
만 쓴다(새 충돌 알고리즘 없음). `base_link`의 충돌 geom은 무명이라
`mj_name2id`가 아니라 `geom_bodyid`로 바디 소속을 찾아 식별했다. 바퀴
geom은 `base_link`이 아니라 별도 바디(`left_wheel_drive` 등)에 속하므로
자동으로 제외된다 — 바퀴-지면 접촉을 오판할 걱정이 없다.

`select_base_pose`는 target 주위 (반경, 각도) 격자로 후보 위치를 만들고,
reachability 점수·발자국 무충돌·현재 위치 근접도 순으로 정렬한다. yaw는
처음엔 "후보 위치에서 target을 정면으로 바라본다"고 가정하려 했으나,
P7.0 기본 격자의 실측 도달 영역이 +x축보다 -y축으로 훨씬 넓게 퍼져
있다는 걸(`y∈[-1.1, 0.2]`) 다시 보고 이 가정을 버렸다 — 어느 로컬 축이
"정면"인지 모델에 새기지 않고, 같은 `candidate_angles` 집합을 위치·방향
양쪽에 재사용해 (위치, 방향) 전체 조합을 탐색하도록 설계했다.

베이스 실행은 `control.whole_body.WholeBodyIK`를 쓰지 않기로 했다 —
이건 손 목표 오차를 줄이는 반응형 솔버라 베이스 이동이 부산물일 뿐,
"지점 (x,y,yaw)로 가라"는 명시적 목표에는 안 맞는다. 대신
`planning/mobile_execution.py`에 `drive_base_to_pose` 헬퍼를 얇게
추가했다 — 매 스텝 월드 오차를 현재 yaw로 회전시켜 차체 프레임 twist를
계산하고, 기존 `control.base.SwerveDrive.update_twist()`에 그대로
넘겨 반환된 바퀴 조향·구동 명령을 `data.ctrl`에 쓴다. 새 저수준 제어는
없다.

## What worked / what failed

**핵심 발견 하나가 이번 cycle의 테스트 설계를 완전히 바꿨다.**
`build_reachability_map`(P7.0)의 docstring은 "checker/solver가 베이스
원점(home 키프레임) 장면이어야 한다"고 요구하는데, 소스를 다시 읽어보니
이 함수는 실제로는 grid point를 **절대 world IK 타겟**으로 그대로 쓴다
— "베이스 원점" 요구사항은 결과를 나중에 상대 좌표로 재해석하기 위한
호출자 쪽 약속일 뿐, 함수 자체는 어떤 base pose로 만든 checker/solver를
줘도 정상 동작한다. 이걸 깨닫고 나서, "먼 베이스 위치에서는 정말로
IK가 실패하고, 재배치 후에는 정말로 성공한다"를 새 IK 코드를 한 줄도
안 짜고 **같은 함수를 두 번(base pose만 다르게) 호출**해서 증명하는
핵심 회귀 테스트를 만들 수 있었다. 처음에 계획했던 "reachability map
점수 차이만 보여주는" 약한 버전보다 훨씬 강한 증거다.

테스트 작성 중 두 번 조정했다:

1. `test_target_unreachable_from_far_base_becomes_reachable_after_repositioning`
   에서 처음엔 `select_base_pose`가 정확히 `(0,0,0)`(yaw까지)을 고를
   거라 기대해 `assert_allclose(result.base_pose, [0,0,0])`로 검증했는데
   실패했다 — 원인: 테스트에 쓴 `ReachabilityMap`이 격자점 1개짜리라
   모든 yaw 후보가 동점(1.0)이 되고, 동점 처리(`key < best_key`, 엄격
   부등호)상 **처음 순회된** 후보가 이긴다. yaw=0이 아니라 후보 각도
   리스트에서 먼저 나온 yaw가 선택된 것 — 버그가 아니라 테스트의
   기대치가 틀렸던 것이다. `(x, y)`만 확인하도록 완화하고, 실제
   도달 가능 여부는 뒤이은 진짜 IK 검증(`near_map`)에 맡겼다.
2. `BaseFootprintChecker`의 "실제 충돌 감지" 테스트를 처음엔 실제
   can-sort 장면의 `table` geom으로 만들려 했으나, `table`은
   `z∈[0.63, 0.73]`인데 `base_link`의 충돌 geom은 절대 z가 고정된
   `[0.27, 0.51]`대라 — 베이스가 아무리 xy로 움직여도 이 장면에선
   물리적으로 절대 충돌할 수 없다는 걸 확인했다(실제 장면엔 이 높이대에
   걸리는 정적 장애물이 하나도 없음). 참-충돌은 작은 합성 MJCF(벽 하나
   + base_link 하나)로 대신 검증하고, 이 사실을 PR 설명에 명시했다.

`ruff check` 통과, 신규 13개 테스트 + 기존 회귀(`test_planning_core.py`
`test_planning_rrt.py` `test_planning_rrt_scene.py`
`test_planning_collision.py` `test_planning_config.py`
`test_planning_shortcut.py` `test_planning_trajectory.py`
`test_planning_reachability.py`) 전부 통과.

## North-star delta

Tier 1(decoupled 모바일 매니퓰레이션) 설계의 두 조각(P7.0 reachability
map, P7.1 베이스 자세 선택+실행)이 모두 구현·테스트 완료 상태다. 이제
"목표가 고정-베이스로 안 닿으면 베이스를 옮겨서 다시 시도"하는 전체
파이프라인의 부품이 갖춰졌다 — 남은 건 사람 리뷰/병합과, 우선순위가
바뀌면 Tier 2(결합형 `WholeBodySpace`)로 확장하는 것뿐이다.

## Key learnings

- **기존 함수의 "의도된 사용법" 제약(docstring)과 "실제로 뭘 하는지"는
  다를 수 있다** — `build_reachability_map`이 "베이스 원점 전용"이라는
  문서 제약은 재사용 편의를 위한 호출자 약속이었지, 함수 내부 로직의
  하드 제약이 아니었다. 소스를 실제로 다시 읽어본 덕분에 새 코드 없이
  더 강한 테스트를 만들 수 있었다 — docstring만 믿지 말고 구현을 직접
  확인하는 습관이 값을 냈다.
- **로봇의 로컬 좌표축이 어느 방향을 "정면"으로 쓰는지 데이터 없이
  가정하면 틀리기 쉽다.** reachability map의 실측 비대칭성(+x보다 -y로
  더 넓음)을 무시하고 "그냥 +x가 정면이겠지"라고 가정했다면 조용히
  틀린 후보를 우선시켰을 것 — 후보 탐색을 축-가정 없이 설계하는 쪽이
  더 안전했다.
- **동점 처리 로직(엄격 부등호로 첫 항목 유지)이 있는 순위 매기기
  함수를 테스트할 땐, 입력(여기선 reachability map)이 실제로 후보들을
  구별할 만큼 충분히 풍부한지 먼저 확인해야 한다** — 격자점 1개짜리
  맵으로는 "정확히 어떤 후보가 선택되는지"를 의미 있게 검증할 수 없다.

## Recommended next 1–3 priorities

1. PR #3/#4/#5/#7/#8/#11 사람 리뷰/병합(여섯 개로 줄었지만 여전히 밀려
   있다). PR #4·#5는 `__init__.py` 충돌 해결 필요(절차 확립됨), PR #11은
   독립적이라 바로 병합 가능.
2. P7 Tier 2(결합형 `WholeBodySpace`) — decoupled 파이프라인이 자리
   잡았으니 우선순위가 바뀌지 않는 한 급하지 않다.
3. 사용자가 우선순위를 정하면 나머지 두 한계(IK 실패 개선 — `MP-0011`;
   연속 동작 부드러움 — `MP-0021`) 중 하나를 이어서 진행.

## Artifacts

- 브랜치: `planning/p7-1-base-pose`(PR #11)
- 신규 파일: `src/ffw_sh5_grasp/planning/base_pose.py`,
  `src/ffw_sh5_grasp/planning/mobile_execution.py`,
  `tests/test_planning_base_pose.py`
- 실측: `test_drive_base_to_pose_reaches_small_target_displacement` —
  home 키프레임에서 0.15m 이동 목표에 실제 물리 스텝(dt=0.001s,
  max_steps=8000)으로 수렴. 신규 13개 + 기존 47개 테스트 전부 통과.
