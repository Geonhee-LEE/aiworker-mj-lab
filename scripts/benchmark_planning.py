"""오른팔 RRT-Connect/RRT* 계획 성공률·시간을 seed 목록에 걸쳐 측정해 TSV에 남긴다.

`results/README.md`의 고정 5컬럼 스키마(timestamp/commit/metric/status/
description)를 그대로 따른다 — seed별 raw 관측치를 `metric` 필드 하나에
인코딩해 append한다(집계는 `scripts/aggregate_results.py` 몫, 여기서는 안 함).

장면은 `scripts/demo_plan_right_arm.py`의 장면 구성·목표 샘플링을 그대로
재사용한다(오브젝트 배치·크기는 실측으로 튜닝된 것이라 중복 구현하지 않는다).
기본은 데모가 시각화용으로 추가한 빨간 구체 장애물 없이, PRD 북극성이 말하는
"진짜" can-sort 장면(상자·테이블·왼팔)만 쓴다 — `--with-obstacle`로 데모와
같은(더 어려운) 구성도 켤 수 있다.

실행 (저장소 루트에서):

    PYTHONPATH=src MUJOCO_GL=osmesa python3 scripts/benchmark_planning.py \
        --seeds 0-49 --out results/p4-benchmark-harness.tsv

    # MP-0017: RRT-Connect 대신 RRT*로 같은 seed를 측정 (비교용)
    PYTHONPATH=src MUJOCO_GL=osmesa python3 scripts/benchmark_planning.py \
        --seeds 0-49 --planner rrt_star --out results/p5-planner-comparison.tsv

    # MP-0007: shortcut 전/후 경로 길이를 둘 다 기록 (path_len/path_len_after)
    PYTHONPATH=src MUJOCO_GL=osmesa python3 scripts/benchmark_planning.py \
        --seeds 0-49 --postprocess shortcut --out results/p5-planner-comparison.tsv

    # MP-0031: 장애물 "배치"를 바꿔가며 RRT-Connect vs RRT* 결론이 얼마나
    # 견고한지 재검증 (같은 50 seed, --obstacle-layout만 바꿈)
    PYTHONPATH=src MUJOCO_GL=osmesa python3 scripts/benchmark_planning.py \
        --seeds 0-49 --with-obstacle --obstacle-layout narrow_passage \
        --planner rrt_star --out results/p5-planner-comparison.tsv

`--obstacle-layout`(``--with-obstacle`` 켰을 때만 의미 있음)은
`OBSTACLE_LAYOUTS`에 정의된 프리셋 중 하나를 고른다 — `"default"`는
기존 3-구체 배치(하위호환), `"narrow_passage"`/`"cluttered"`는 이
스크립트로 직접 실측 검증한 더 어려운 배치다(반복 횟수 분포가 뚜렷이
넓게 퍼짐 — 각 프리셋 정의부 주석 참고). `--with-obstacle`은 여전히
장애물 on/off 스위치이고, `--obstacle-layout`은 그와 별개로 "켰을 때
어떤 배치를 쓸지"를 고르는 축이다.

`--planner rrt_star`는 `--goal-bias`/`--time-budget-s`의 기본값이
`--planner rrt_connect`와 다르다(각각 0.3/15.0 vs 0.1/5.0) —
`demo_plan_right_arm.py`가 이미 쓰는 것과 같은 이유(단일 트리 RRT*는
bidirectional CONNECT가 없어 목표 쪽으로 매 반복 조금씩만 전진한다)다.
다만 시간 예산 자체는 데모 권장값(30초)보다 짧다(15초) — 50-seed
벤치마크 총 소요 시간을 실용적 범위로 낮추기 위한 벤치마크 전용 선택.

`--wall-budget-s`(기본 100초, PRD의 2분 요구사항에서 여유를 둠)를 넘기면
남은 seed는 건너뛰고 조용히 멈춘다 — 자동화 루프가 절대 예산을 넘기지
않게 하기 위한 안전장치다(R-NF-002). `--planner rrt_star`는 seed당 최대
15초까지 걸릴 수 있어 50 seed 전체를 다 돌리려면 `--wall-budget-s`를
그만큼 넉넉히(예: 900+) 올려야 한다.
"""

import argparse
import datetime
import pathlib
import subprocess
import sys
import time

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from demo_plan_right_arm import (  # noqa: E402
    BASE_REQUIRE_CONTACT_GEOMS,
    DEFAULT_START,
    OBSTACLE_PREFIX,
    OBSTACLE_SPHERES,
    _build_scene,
    _sample_valid_goal,
)

from ffw_sh5_grasp.planning import (  # noqa: E402
    ArmCollisionChecker,
    EdgeChecker,
    RightArmSpace,
    plan_rrt_connect,
    plan_rrt_star,
    shortcut_path,
)

RESULTS_HEADER = "timestamp\tcommit\tmetric\tstatus\tdescription"
# 장애물 "배치"를 다양화하기 위한 프리셋. ``None``은 demo의 기본
# ``OBSTACLE_SPHERES``를 그대로 쓴다는 뜻(하위호환 — 기존 --with-obstacle
# 동작과 완전히 동일). 각 (x, y, z, radius) 좌표는 이 스크립트로 직접
# 실측 검증했다 — ``DEFAULT_START``가 유효하고, RRT-Connect가 여전히
# 대부분 성공하되(우연한 실패로 벤치마크가 무의미해지지 않도록) 반복
# 횟수 분포가 "default"보다 뚜렷이 넓게 퍼지는(=실제로 더 어려운) 조합만
# 채택했다.
OBSTACLE_LAYOUTS = {
    "default": OBSTACLE_SPHERES,
    # 손끝이 자주 지나는 좁은 구간(중앙값 근방)에 지름 14cm 틈만 남긴
    # 관문 2개 — 20-seed 실측: 100% 성공하지만 반복 횟수 1~172회로
    # "default"(대개 1~20회)보다 훨씬 넓게 퍼진다.
    "narrow_passage": (
        (0.15, -0.65, 1.3, 0.07),
        (-0.15, -0.65, 1.3, 0.07),
    ),
    # 기존 3개 구체에 3개를 더 흩뿌린 6개 배치 — 20-seed 실측: 100% 성공,
    # 반복 횟수 1~204회(중앙값이 눈에 띄게 높아짐, 1회 즉시 성공 사례가
    # "default"보다 드물다).
    "cluttered": (
        (-0.05, -0.91, 1.61, 0.06),
        (0.55, -0.81, 1.19, 0.06),
        (-0.01, -0.80, 1.17, 0.06),
        (0.25, -0.95, 1.35, 0.06),
        (-0.30, -0.85, 1.05, 0.06),
        (0.10, -0.70, 0.90, 0.06),
    ),
}
# ``format_metric``의 ``path_len_after``가 "레거시 호출이라 필드 자체를
# 생략"과 "postprocess는 켰지만 이 seed는 실패해서 NA"를 구분하기 위한
# 표식. ``None``은 이미 "실패해서 NA"라는 뜻으로 쓰이고 있어 재사용할 수
# 없다.
_NOT_PROVIDED = object()


def parse_seed_spec(spec):
    """``"0-49"``/``"0,3,7"``/``"0-9,20,30-39"``를 정렬된 seed 정수 리스트로."""
    seeds = set()
    for token in spec.split(","):
        token = token.strip()
        if not token:
            continue
        if "-" in token:
            low, high = token.split("-", 1)
            seeds.update(range(int(low), int(high) + 1))
        else:
            seeds.add(int(token))
    return sorted(seeds)


def format_metric(
    seed, success, plan_ms, iterations, checks, path_len,
    *, planner=None, path_len_after=_NOT_PROVIDED,
):
    """``bench:seed=<n>[,planner=<p>],success=<0|1>,plan_ms=<x>,iterations=<i>,checks=<c>,path_len=<l|NA>[,path_len_after=<l|NA>]``.

    ``planner``/``path_len_after``는 둘 다 선택적이다 — 생략하면(둘 다
    기본값) 이전 버전과 바이트 단위로 동일한 문자열이 나온다(하위호환).
    ``planner``를 넘기면 ``seed=`` 바로 뒤에 끼워 넣어 "이 행이 어느
    플래너로 계획됐는지" 표시한다. ``path_len_after``는 shortcut 등
    후처리 후 경로 길이용 — ``_NOT_PROVIDED``(기본)면 후처리를 아예 안 켠
    실행이라 필드 자체를 생략하고, 값(``None`` 포함)을 넘기면 켰다는
    뜻이라 항상 필드를 찍는다(``None``은 "후처리는 켰지만 이 seed는
    계획 자체가 실패해 후처리할 경로가 없었다"는 뜻으로 "NA"가 된다).
    """
    path_len_str = f"{path_len:.3f}" if path_len is not None else "NA"
    planner_part = f",planner={planner}" if planner is not None else ""
    metric = (
        f"bench:seed={seed}{planner_part},success={int(success)},plan_ms={plan_ms:.2f},"
        f"iterations={iterations},checks={checks},path_len={path_len_str}"
    )
    if path_len_after is not _NOT_PROVIDED:
        after_str = f"{path_len_after:.3f}" if path_len_after is not None else "NA"
        metric += f",path_len_after={after_str}"
    return metric


def _path_length_rad(space, path):
    path = np.asarray(path, dtype=float)
    if len(path) < 2:
        return 0.0
    return float(sum(space.distance(path[i], path[i + 1]) for i in range(len(path) - 1)))


def _plan(space, edge_checker, start, goal, rng, planner, planner_kwargs):
    """``planner``(``"rrt_connect"``/``"rrt_star"``)에 따라 계획 함수를 고른다.

    ``scripts/demo_plan_right_arm.py``의 ``_plan_path``와 같은 분기 패턴 —
    다만 스크립트끼리는 서로 import하지 않는 기존 관례를 따라 이 파일
    안에 자체 구현한다. 둘 다 같은 ``PlannerResult``를 반환하므로 호출자
    쪽 코드는 어느 플래너를 골랐는지 신경 쓸 필요가 없다.
    """
    if planner == "rrt_star":
        return plan_rrt_star(space, edge_checker, start, goal, rng=rng, **planner_kwargs)
    return plan_rrt_connect(space, edge_checker, start, goal, rng=rng, **planner_kwargs)


def run_benchmark(
    seeds,
    *,
    with_obstacle,
    padding_m,
    step_size_rad,
    goal_bias,
    max_iterations,
    time_budget_s,
    wall_budget_s,
    planner="rrt_connect",
    postprocess="none",
    goal_tolerance_rad=None,
    rewire_radius_rad=None,
    obstacle_layout="default",
):
    """장면을 한 번 만들고, seed마다 목표를 샘플링해 ``planner``로 1회 계획한다.

    ``wall_budget_s``(전체 벽시계 예산)를 넘기면 그 시점에서 멈추고 그때까지
    쌓인 행만 반환한다 — 나머지 seed는 이번 실행에서 아예 시도하지 않는다.

    ``postprocess="shortcut"``이면 성공한 경로에 ``shortcut_path``를 적용해
    raw 길이(``path_len``)와 후처리 길이(``path_len_after``)를 둘 다
    기록한다(MP-0007: 실제 장면에서 shortcut이 경로를 얼마나 줄이는지).

    ``obstacle_layout``(``OBSTACLE_LAYOUTS`` 키, ``with_obstacle=True``일
    때만 의미 있음)으로 장애물 "배치"를 바꿀 수 있다 — 같은 on/off
    스위치(``with_obstacle``)와 별개 축이라, 배치에 따라 RRT-Connect
    vs RRT* 결론이 달라지는지(MP-0031) 확인하는 데 쓴다.
    """
    spheres = OBSTACLE_LAYOUTS[obstacle_layout]
    model, data = _build_scene(with_obstacle=with_obstacle, with_marker=False, spheres=spheres)
    space = RightArmSpace.from_model(model)
    if with_obstacle:
        obstacle_names = tuple(f"{OBSTACLE_PREFIX}{i}" for i in range(len(spheres)))
        require_contact_geoms = BASE_REQUIRE_CONTACT_GEOMS + obstacle_names
    else:
        require_contact_geoms = BASE_REQUIRE_CONTACT_GEOMS
    checker = ArmCollisionChecker(
        model, space, padding_m=padding_m, require_contact_geoms=require_contact_geoms
    )
    checker.set_snapshot(data)
    edge_checker = EdgeChecker(space, checker.is_valid, resolution_rad=0.05)

    if not checker.is_valid(DEFAULT_START):
        raise RuntimeError("DEFAULT_START가 이 장면 설정에서 무효합니다")

    rows = []
    wall_start = time.perf_counter()
    for seed in seeds:
        if time.perf_counter() - wall_start > wall_budget_s:
            print(
                f"wall_budget_s={wall_budget_s}s 초과 — seed {seed} 이후 "
                f"{len(seeds) - len(rows)}개 건너뜀",
                file=sys.stderr,
            )
            break

        rng = np.random.default_rng(seed)
        goal = _sample_valid_goal(space, checker, rng)

        planner_kwargs = {
            "step_size_rad": step_size_rad, "goal_bias": goal_bias,
            "max_iterations": max_iterations, "time_budget_s": time_budget_s,
        }
        if planner == "rrt_star":
            planner_kwargs["rewire_radius_rad"] = rewire_radius_rad
            planner_kwargs["goal_tolerance_rad"] = goal_tolerance_rad

        query_start = time.perf_counter()
        result = _plan(space, edge_checker, DEFAULT_START, goal, rng, planner, planner_kwargs)
        plan_ms = (time.perf_counter() - query_start) * 1000.0

        path_len = _path_length_rad(space, result.path) if result.success else None

        metric_kwargs = {"planner": planner}
        if postprocess == "shortcut":
            path_len_after = None
            if result.success:
                smoothed = shortcut_path(space, edge_checker, result.path, rng=rng, iterations=200)
                path_len_after = _path_length_rad(space, smoothed)
            metric_kwargs["path_len_after"] = path_len_after

        rows.append(
            format_metric(
                seed, result.success, plan_ms, result.iterations,
                result.state_checks, path_len, **metric_kwargs,
            )
        )

    return rows


def append_tsv(path, metric_rows, *, commit, status, description):
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    is_new = not path.exists() or path.stat().st_size == 0
    now = datetime.datetime.now().astimezone().isoformat(timespec="seconds")
    with path.open("a", encoding="utf-8") as handle:
        if is_new:
            handle.write(RESULTS_HEADER + "\n")
        for metric in metric_rows:
            handle.write(f"{now}\t{commit}\t{metric}\t{status}\t{description}\n")


def _git_commit():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], cwd=REPO_ROOT, text=True,
        ).strip()
    except Exception:
        return "unknown"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", required=True, help='예: "0-49" 또는 "0,3,7"')
    parser.add_argument("--out", required=True, help="results/<phase>-<slug>.tsv")
    parser.add_argument("--with-obstacle", action="store_true", help="데모용 빨간 구체 장애물도 포함(더 어려운 구성)")
    parser.add_argument(
        "--obstacle-layout", choices=list(OBSTACLE_LAYOUTS), default="default",
        help="--with-obstacle일 때만 의미 있음. 장애물 '배치' 자체를 바꿔 RRT-Connect vs RRT* 결론이 배치에 따라 달라지는지 확인(MP-0031)",
    )
    parser.add_argument("--padding-m", type=float, default=0.012)
    parser.add_argument("--step-size-rad", type=float, default=0.3)
    parser.add_argument(
        "--planner", choices=["rrt_connect", "rrt_star"], default="rrt_connect",
        help="demo_plan_right_arm.py --planner와 동일. rrt_star는 시간 예산이 끝날 때까지 계속 개선한다",
    )
    parser.add_argument(
        "--postprocess", choices=["none", "shortcut"], default="none",
        help="shortcut이면 raw/후처리 경로 길이를 둘 다 기록한다(MP-0007: 실제 감소폭 측정)",
    )
    parser.add_argument(
        "--goal-bias", type=float, default=None,
        help="미지정 시 rrt_connect=0.1, rrt_star=0.3 (demo_plan_right_arm.py와 동일한 이유)",
    )
    parser.add_argument("--max-iterations", type=int, default=4000)
    parser.add_argument(
        "--time-budget-s", type=float, default=None,
        help="질의(seed) 하나당 예산. 미지정 시 rrt_connect=5.0, rrt_star=15.0(벤치마크 전용 — demo 권장 30초보다 짧게 잡아 50-seed 총 실행 시간을 실용적 범위로 낮춘다)",
    )
    parser.add_argument(
        "--goal-tolerance-rad", type=float, default=None,
        help="rrt_star 전용. 미지정 시 0.5",
    )
    parser.add_argument(
        "--rewire-radius-rad", type=float, default=None,
        help="rrt_star 전용. 미지정 시 2 * --step-size-rad",
    )
    parser.add_argument("--wall-budget-s", type=float, default=100.0, help="전체 실행 예산")
    parser.add_argument("--description", default="RRT-Connect can-sort 성공률 벤치마크")
    parser.add_argument("--status", default="keep", choices=["keep", "discard", "crash", "in_progress"])
    args = parser.parse_args(argv)

    if args.goal_bias is None:
        args.goal_bias = 0.3 if args.planner == "rrt_star" else 0.1
    if args.time_budget_s is None:
        args.time_budget_s = 15.0 if args.planner == "rrt_star" else 5.0
    if args.goal_tolerance_rad is None:
        args.goal_tolerance_rad = 0.5
    if args.rewire_radius_rad is None:
        args.rewire_radius_rad = 2.0 * args.step_size_rad

    seeds = parse_seed_spec(args.seeds)
    print(
        f"seed {len(seeds)}개, planner={args.planner}, postprocess={args.postprocess}, "
        f"with_obstacle={args.with_obstacle}, obstacle_layout={args.obstacle_layout}, "
        f"wall_budget_s={args.wall_budget_s}"
    )

    wall_start = time.perf_counter()
    metric_rows = run_benchmark(
        seeds,
        with_obstacle=args.with_obstacle,
        padding_m=args.padding_m,
        step_size_rad=args.step_size_rad,
        goal_bias=args.goal_bias,
        max_iterations=args.max_iterations,
        time_budget_s=args.time_budget_s,
        wall_budget_s=args.wall_budget_s,
        planner=args.planner,
        postprocess=args.postprocess,
        goal_tolerance_rad=args.goal_tolerance_rad,
        rewire_radius_rad=args.rewire_radius_rad,
        obstacle_layout=args.obstacle_layout,
    )
    elapsed = time.perf_counter() - wall_start

    append_tsv(
        args.out, metric_rows,
        commit=_git_commit(), status=args.status, description=args.description,
    )

    successes = sum(1 for row in metric_rows if ",success=1," in row)
    print(
        f"완료: {len(metric_rows)}/{len(seeds)} seed 실행, 성공 {successes}/{len(metric_rows)}, "
        f"벽시계 {elapsed:.1f}s → {args.out}"
    )


if __name__ == "__main__":
    main()
