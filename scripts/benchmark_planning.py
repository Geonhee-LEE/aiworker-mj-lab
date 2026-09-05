"""오른팔 RRT-Connect 계획 성공률·시간을 seed 목록에 걸쳐 측정해 TSV에 남긴다.

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

`--wall-budget-s`(기본 100초, PRD의 2분 요구사항에서 여유를 둠)를 넘기면
남은 seed는 건너뛰고 조용히 멈춘다 — 자동화 루프가 절대 예산을 넘기지
않게 하기 위한 안전장치다(R-NF-002).
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
    DEFAULT_START,
    OBSTACLE_NAMES,
    REQUIRE_CONTACT_GEOMS,
    _build_scene,
    _sample_valid_goal,
)

from ffw_sh5_grasp.planning import (  # noqa: E402
    ArmCollisionChecker,
    EdgeChecker,
    RightArmSpace,
    plan_rrt_connect,
)

RESULTS_HEADER = "timestamp\tcommit\tmetric\tstatus\tdescription"


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


def format_metric(seed, success, plan_ms, iterations, checks, path_len):
    """``bench:seed=<n>,success=<0|1>,plan_ms=<x>,iterations=<i>,checks=<c>,path_len=<l|NA>``."""
    path_len_str = f"{path_len:.3f}" if path_len is not None else "NA"
    return (
        f"bench:seed={seed},success={int(success)},plan_ms={plan_ms:.2f},"
        f"iterations={iterations},checks={checks},path_len={path_len_str}"
    )


def _path_length_rad(space, path):
    path = np.asarray(path, dtype=float)
    if len(path) < 2:
        return 0.0
    return float(sum(space.distance(path[i], path[i + 1]) for i in range(len(path) - 1)))


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
):
    """장면을 한 번 만들고, seed마다 목표를 샘플링해 RRT-Connect 1회 계획한다.

    ``wall_budget_s``(전체 벽시계 예산)를 넘기면 그 시점에서 멈추고 그때까지
    쌓인 행만 반환한다 — 나머지 seed는 이번 실행에서 아예 시도하지 않는다.
    """
    model, data = _build_scene(with_obstacle=with_obstacle, with_marker=False)
    space = RightArmSpace.from_model(model)
    require_contact_geoms = (
        REQUIRE_CONTACT_GEOMS if with_obstacle
        else tuple(name for name in REQUIRE_CONTACT_GEOMS if name not in OBSTACLE_NAMES)
    )
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

        query_start = time.perf_counter()
        result = plan_rrt_connect(
            space, edge_checker, DEFAULT_START, goal,
            rng=rng, step_size_rad=step_size_rad, goal_bias=goal_bias,
            max_iterations=max_iterations, time_budget_s=time_budget_s,
        )
        plan_ms = (time.perf_counter() - query_start) * 1000.0

        path_len = _path_length_rad(space, result.path) if result.success else None
        rows.append(
            format_metric(
                seed, result.success, plan_ms, result.iterations,
                result.state_checks, path_len,
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
    parser.add_argument("--padding-m", type=float, default=0.012)
    parser.add_argument("--step-size-rad", type=float, default=0.3)
    parser.add_argument("--goal-bias", type=float, default=0.1)
    parser.add_argument("--max-iterations", type=int, default=4000)
    parser.add_argument("--time-budget-s", type=float, default=5.0, help="질의(seed) 하나당 예산")
    parser.add_argument("--wall-budget-s", type=float, default=100.0, help="전체 실행 예산")
    parser.add_argument("--description", default="RRT-Connect can-sort 성공률 벤치마크")
    parser.add_argument("--status", default="keep", choices=["keep", "discard", "crash", "in_progress"])
    args = parser.parse_args(argv)

    seeds = parse_seed_spec(args.seeds)
    print(f"seed {len(seeds)}개, with_obstacle={args.with_obstacle}, wall_budget_s={args.wall_budget_s}")

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
