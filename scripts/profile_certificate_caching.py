"""safety-certificate 스타일 충돌 검사 캐싱 도입 여부를 재는 프로파일러 (MP-0028).

`research/2026-09/005.md`에서 조사한 아이디어(Bialkowski et al., IJRR 2016):
한 번 `clearance(q)`로 잰 최소 거리를 반경으로 저장해두면, 이후 질의점이 그
반경 안에 들어오면 `is_valid`를 생략할 수 있다. 다만 이 저장소는 반경을
얻으려면 `is_valid`보다 훨씬 비싼 `clearance()`를 호출해야 하므로,
"인증서 1개가 아껴주는 is_valid 호출 수"가 "clearance 1회의 상대 비용"보다
커야 순이득이다. 이 스크립트는 그 비율을 실측만 한다 — 캐싱 자체는
구현하지 않는다(바로 구현하지 않기로 한 스코프 결정, 위 문서 참고).

방법론:
1. 실제 can-sort 장면에서 RRT-Connect를 몇 seed 돌려, 트리 확장·edge 검사
   과정에서 실제로 `is_valid`에 넘겨진 configuration들을 수집한다(균등 무작위
   표본이 아니라 플래너가 실제로 방문하는 분포를 쓴다).
2. 그 표본에서 무작위로 부분집합을 뽑아 `is_valid`와 `clearance`를 각각
   반복 호출해 호출당 중앙값 시간을 잰다.
3. 같은 표본의 `clearance()` 값(=인증서 반경 후보) 분포를 `resolution_rad`
   (저장소 관례값 0.05, `benchmark_planning.py`와 동일)와 비교한다.
4. "인증서 하나가 같은 edge 위 이웃 waypoint들을 반경만큼 덮는다"는 낙관적
   가정 아래 기대 절감 호출 수(`2 * clearance / resolution_rad`)를
   `clearance/is_valid` 비용비와 비교해 손익분기 여부를 판정한다.

실행 (저장소 루트에서, 2분 예산 이내):

    PYTHONPATH=src MUJOCO_GL=osmesa python3 scripts/profile_certificate_caching.py \
        --seeds 0-4 --out results/p1-safety-certificate-profiling.tsv
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
RESOLUTION_RAD = 0.05  # benchmark_planning.py와 동일한 저장소 관례값


def parse_seed_spec(spec):
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


def collect_visited_configs(space, checker, edge_checker, seeds, *, max_iterations, time_budget_s):
    """RRT-Connect가 실제로 `is_valid`에 넘긴 configuration들을 모은다."""
    visited = []
    real_is_valid = checker.is_valid

    def recording_is_valid(q, **kwargs):
        visited.append(np.array(q, dtype=float))
        return real_is_valid(q, **kwargs)

    checker.is_valid = recording_is_valid
    edge_checker.is_valid = recording_is_valid
    try:
        for seed in seeds:
            rng = np.random.default_rng(seed)
            goal = _sample_valid_goal(space, checker, rng)
            plan_rrt_connect(
                space, edge_checker, DEFAULT_START, goal,
                rng=rng, step_size_rad=0.3, goal_bias=0.1,
                max_iterations=max_iterations, time_budget_s=time_budget_s,
            )
    finally:
        checker.is_valid = real_is_valid
        edge_checker.is_valid = real_is_valid
    return visited


def _median_call_time_ms(fn, samples, *, repeats):
    """샘플마다 `repeats`회 반복 호출해 호출당 중앙값 시간(ms)을 잰다."""
    for q in samples[: max(1, len(samples) // 10)]:
        fn(q)  # 워밍업(첫 호출의 페이지폴트/캐시 미스 영향 배제)
    per_call = []
    for q in samples:
        start = time.perf_counter()
        for _ in range(repeats):
            fn(q)
        elapsed = time.perf_counter() - start
        per_call.append(elapsed / repeats * 1000.0)
    return float(np.median(per_call))


def run_profile(seeds, *, with_obstacle, padding_m, sample_size, timing_repeats, rng_seed):
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
    edge_checker = EdgeChecker(space, checker.is_valid, resolution_rad=RESOLUTION_RAD)

    visited = collect_visited_configs(
        space, checker, edge_checker, seeds, max_iterations=4000, time_budget_s=5.0,
    )
    if not visited:
        raise RuntimeError("RRT-Connect가 어떤 is_valid 호출도 남기지 않았습니다")

    rng = np.random.default_rng(rng_seed)
    sample_idx = rng.choice(len(visited), size=min(sample_size, len(visited)), replace=False)
    samples = [visited[i] for i in sample_idx]

    is_valid_ms = _median_call_time_ms(checker.is_valid, samples, repeats=timing_repeats)
    clearance_ms = _median_call_time_ms(checker.clearance, samples, repeats=timing_repeats)
    cost_ratio = clearance_ms / is_valid_ms if is_valid_ms > 0 else float("inf")

    clearance_values = np.array([checker.clearance(q) for q in samples], dtype=float)
    median_clearance = float(np.median(clearance_values))
    p10_clearance = float(np.percentile(clearance_values, 10))

    # 낙관적 상한: 인증서 반경 안에 들어오는 같은 edge 위 이웃 waypoint 개수.
    expected_savings = 2.0 * median_clearance / RESOLUTION_RAD
    worth_it = expected_savings > cost_ratio

    return {
        "n_visited": len(visited),
        "n_sampled": len(samples),
        "is_valid_ms": is_valid_ms,
        "clearance_ms": clearance_ms,
        "cost_ratio": cost_ratio,
        "median_clearance_rad": median_clearance,
        "p10_clearance_rad": p10_clearance,
        "resolution_rad": RESOLUTION_RAD,
        "expected_savings": expected_savings,
        "worth_it": worth_it,
    }


def format_metric(result):
    verdict = "worth-poc" if result["worth_it"] else "not-worth"
    return (
        f"qual:cost_ratio={result['cost_ratio']:.1f},"
        f"is_valid_ms={result['is_valid_ms']:.4f},"
        f"clearance_ms={result['clearance_ms']:.4f},"
        f"median_clearance_rad={result['median_clearance_rad']:.4f},"
        f"p10_clearance_rad={result['p10_clearance_rad']:.4f},"
        f"resolution_rad={result['resolution_rad']:.3f},"
        f"expected_savings={result['expected_savings']:.1f},"
        f"n_sampled={result['n_sampled']},"
        f"verdict={verdict}"
    )


def append_tsv(path, metric, *, commit, status, description):
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    is_new = not path.exists() or path.stat().st_size == 0
    now = datetime.datetime.now().astimezone().isoformat(timespec="seconds")
    with path.open("a", encoding="utf-8") as handle:
        if is_new:
            handle.write(RESULTS_HEADER + "\n")
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
    parser.add_argument("--seeds", default="0-4", help='RRT-Connect 방문 표본을 모을 seed, 예: "0-4"')
    parser.add_argument("--out", required=True, help="results/<phase>-<slug>.tsv")
    parser.add_argument("--with-obstacle", action="store_true", help="데모용 장애물 포함(더 빡빡한 clearance)")
    parser.add_argument("--padding-m", type=float, default=0.012)
    parser.add_argument("--sample-size", type=int, default=150)
    parser.add_argument("--timing-repeats", type=int, default=20, help="샘플당 반복 호출 수(타이밍 노이즈 완화)")
    parser.add_argument("--rng-seed", type=int, default=0)
    parser.add_argument("--description", default="safety-certificate 캐싱 순이득 여부 프로파일링(MP-0028)")
    parser.add_argument("--status", default="keep", choices=["keep", "discard", "crash", "in_progress"])
    args = parser.parse_args(argv)

    seeds = parse_seed_spec(args.seeds)
    wall_start = time.perf_counter()
    result = run_profile(
        seeds,
        with_obstacle=args.with_obstacle,
        padding_m=args.padding_m,
        sample_size=args.sample_size,
        timing_repeats=args.timing_repeats,
        rng_seed=args.rng_seed,
    )
    elapsed = time.perf_counter() - wall_start

    metric = format_metric(result)
    append_tsv(args.out, metric, commit=_git_commit(), status=args.status, description=args.description)

    print(
        f"완료: is_valid={result['is_valid_ms']:.4f}ms clearance={result['clearance_ms']:.4f}ms "
        f"(비율 {result['cost_ratio']:.1f}x), median_clearance={result['median_clearance_rad']:.4f}rad "
        f"(resolution_rad={result['resolution_rad']:.3f}), 기대 절감={result['expected_savings']:.1f}회 "
        f"→ {'worth-poc' if result['worth_it'] else 'not-worth'}, 벽시계 {elapsed:.1f}s → {args.out}"
    )


if __name__ == "__main__":
    main()
