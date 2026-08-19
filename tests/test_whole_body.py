"""ROS 없는 모바일 기구학과 전신 IK 회귀 검사.

Headless 실행: ``python3 tests/test_whole_body.py``
"""

import ast
import itertools
import pathlib
import sys
import time

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
MODEL_PATH = REPO_ROOT / "models" / "full_scene.xml"

from ffw_sh5_grasp.application import targets as teleop_targets  # noqa: E402
from ffw_sh5_grasp.control import arm as arm_control  # noqa: E402
from ffw_sh5_grasp.control import base as base_teleop  # noqa: E402
from ffw_sh5_grasp.control import bimanual  # noqa: E402
from ffw_sh5_grasp.control import grasp  # noqa: E402
from ffw_sh5_grasp.control import whole_body as whole_body_ik  # noqa: E402
from ffw_sh5_grasp.kinematics import optimization as bounded_optimization  # noqa: E402
from ffw_sh5_grasp.kinematics import rotations as kinematics_math  # noqa: E402
from ffw_sh5_grasp.kinematics import tasks as pose_tasks  # noqa: E402
import kinematics  # noqa: E402
import teleop_app  # noqa: E402

ARMS = {side: [f"arm_{side}_joint{i}" for i in range(1, 8)] for side in ("r", "l")}
HOME = np.array([0.0, 0.0, 0.0, -np.pi / 2, 0.0, 0.0, 0.0])
ORIENTATION_ERROR_LENGTH = 0.20


def run_ros_free_dependency_gate():
    """런타임 경로에 ROS나 MoveIt 의존성이 실수로 들어오지 않게 검사한다."""
    runtime_files = (
        "ffw_sh5_grasp/control/base.py",
        "ffw_sh5_grasp/control/bimanual.py",
        "ffw_sh5_grasp/control/whole_body.py",
        "ffw_sh5_grasp/kinematics/optimization.py",
        "ffw_sh5_grasp/kinematics/constraints.py",
        "ffw_sh5_grasp/kinematics/solver.py",
        "ffw_sh5_grasp/kinematics/tasks.py",
        "ffw_sh5_grasp/kinematics/rotations.py",
        "ffw_sh5_grasp/kinematics/tree.py",
        "ffw_sh5_grasp/kinematics/collision.py",
        "ffw_sh5_grasp/application/targets.py",
        "ffw_sh5_grasp/application/teleop.py")
    forbidden = {
        "rclpy", "rospy", "geometry_msgs", "nav_msgs", "sensor_msgs", "tf2_ros",
        "moveit", "moveit_commander", "ament_index_python", "controller_manager",
    }
    imported = set()
    for filename in runtime_files:
        source = (REPO_ROOT / "src" / filename).read_text(encoding="utf-8")
        for node in ast.walk(ast.parse(source, filename=filename)):
            if isinstance(node, ast.Import):
                imported.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.add(node.module.split(".")[0])
    violations = sorted(imported & forbidden)
    ok = not violations
    print(f"ROS-free dependency gate: forbidden imports={violations}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_tree_kinematics_dependency_gate():
    """런타임의 모든 손 FK/Jacobian이 자체 기구학 트리를 사용하는지 검사한다."""
    runtime_files = (
        "ffw_sh5_grasp/kinematics/optimization.py",
        "ffw_sh5_grasp/kinematics/constraints.py",
        "ffw_sh5_grasp/kinematics/solver.py",
        "ffw_sh5_grasp/kinematics/tasks.py",
        "ffw_sh5_grasp/kinematics/rotations.py",
        "ffw_sh5_grasp/kinematics/tree.py",
        "ffw_sh5_grasp/kinematics/collision.py",
        "ffw_sh5_grasp/control/bimanual.py",
        "ffw_sh5_grasp/control/whole_body.py",
        "ffw_sh5_grasp/application/targets.py",
        "ffw_sh5_grasp/application/teleop.py")
    modules = {}
    for filename in runtime_files:
        source = (REPO_ROOT / "src" / filename).read_text(encoding="utf-8")
        modules[filename] = ast.parse(source, filename=filename)
    # 구현 분리 후에도 tree와 solver가 각각 의도한 모듈에 있어야 한다.
    expected_classes = {
        "ffw_sh5_grasp/kinematics/tree.py": "KinematicTree",
        "ffw_sh5_grasp/kinematics/solver.py": "DifferentialIKSolver",
    }
    solver_classes = {
        filename: {
            node.name for node in modules[filename].body
            if isinstance(node, ast.ClassDef)
        }
        for filename in expected_classes
    }
    solver_forbidden = {"MjData", "mj_forward"}
    runtime_forbidden = {
        "mj_forward", "mj_jacSite", "mj_jac", "site_xpos", "site_xmat"}
    violations = set()
    for filename, parsed in modules.items():
        for child in ast.walk(parsed):
            if not isinstance(child, ast.Attribute):
                continue
            is_mujoco_call = (
                isinstance(child.value, ast.Name) and child.value.id == "mujoco")
            if child.attr in runtime_forbidden:
                violations.add(f"{filename}:{child.attr}")
            if filename in {
                    "ffw_sh5_grasp/kinematics/solver.py",
                    "ffw_sh5_grasp/kinematics/tree.py"} and is_mujoco_call:
                if child.attr in solver_forbidden:
                    violations.add(f"{filename}:{child.attr}")
    violations = sorted(violations)
    classes_ok = all(
        class_name in solver_classes[filename]
        for filename, class_name in expected_classes.items())
    expected_functions = {
        "ffw_sh5_grasp/kinematics/optimization.py": {
            "least_squares_to_qp", "bounded_quadratic_program",
            "bounded_quadratic_program_with_barriers"},
        "ffw_sh5_grasp/kinematics/constraints.py": {
            "joint_velocity_bounds", "collision_velocity_barriers",
            "clip_joint_positions"},
        "ffw_sh5_grasp/control/bimanual.py": {
            "capture_reference", "rigid_grasp_task"},
    }
    functions_ok = all(
        function_names <= {
            node.name for node in modules[filename].body
            if isinstance(node, ast.FunctionDef)
        }
        for filename, function_names in expected_functions.items()
    )
    ok = classes_ok and functions_ok and not violations
    print(f"Tree-kinematics dependency gate: forbidden runtime FK={violations}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def _reset(model, data):
    """전신 모델을 home 키프레임으로 초기화하고 모든 파생 상태를 갱신한다."""
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)


def _sites(model):
    """오른손과 왼손 grasp target site ID를 side 키 사전으로 반환한다."""
    return {
        side: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"grasp_target_{side}")
        for side in ("r", "l")
    }


def _target_poses(data, sites, delta):
    """현재 양손 pose에 공통 위치 증분을 더한 WBIK 목표 사전을 만든다."""
    targets = {}
    for side, site_id in sites.items():
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, data.site_xmat[site_id])
        targets[side] = (data.site_xpos[site_id].copy() + np.asarray(delta), quat)
    return targets


def _pose_error_metric(data, targets, sites):
    """위치 오차와 자세 오차를 20 cm 등가 지레팔 기준으로 합산한다."""
    total = 0.0
    for side, site in sites.items():
        target_pos, target_quat = targets[side]
        current_quat = np.zeros(4)
        mujoco.mju_mat2Quat(current_quat, data.site_xmat[site])
        orientation_error = np.zeros(3)
        mujoco.mju_subQuat(orientation_error, target_quat, current_quat)
        total += np.linalg.norm(target_pos - data.site_xpos[site])
        total += ORIENTATION_ERROR_LENGTH * np.linalg.norm(orientation_error)
    return float(total)


def run_shared_pose_task_gate():
    """세 IK 경로가 공유하는 pose 오차와 속도 제한 규칙을 검사한다."""
    current_position = np.array([0.2, -0.1, 0.5])
    target_position = np.array([0.5, 0.3, 0.4])
    current_quaternion = kinematics_math.rpy_deg_to_quat([10.0, -5.0, 20.0])
    target_quaternion = kinematics_math.rpy_deg_to_quat([-15.0, 8.0, 70.0])
    error = pose_tasks.pose_error(
        current_position, current_quaternion,
        target_position, target_quaternion)
    same_rotation_error = pose_tasks.pose_error(
        current_position, -current_quaternion,
        target_position, -target_quaternion)
    command = pose_tasks.pose_velocity_command(
        error,
        position_gain=4.0,
        orientation_gain=3.0,
        current_twist=np.array([0.1, -0.2, 0.3, -0.1, 0.2, -0.3]),
        linear_velocity_damping=0.5,
        angular_velocity_damping=0.25,
        max_linear_speed=0.7,
        max_angular_speed=1.1,
    )
    position_ok = np.allclose(
        error.position, target_position - current_position)
    double_cover_ok = np.allclose(
        error.orientation, same_rotation_error.orientation, atol=1e-12)
    bounded = (
        np.linalg.norm(command[:3]) <= 0.7 + 1e-12
        and np.linalg.norm(command[3:]) <= 1.1 + 1e-12
    )
    ok = position_ok and double_cover_ok and bounded
    print(f"Shared pose-task gate: position={position_ok} "
          f"quaternion_double_cover={double_cover_ok} bounded={bounded}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_swerve_kinematics_gate():
    """스워브 역·정기구학 왕복, 조향 실행 가능성과 포화 비율을 검사한다."""
    kin = base_teleop.SwerveKinematics()
    limited_kin = base_teleop.SwerveKinematics(steer_range=(-1.58, 1.58))
    rng = np.random.default_rng(42)
    max_roundtrip = 0.0
    feasible = True
    for _ in range(100):
        twist = base_teleop.BodyTwist(*rng.uniform([-0.4, -0.4, -0.7], [0.4, 0.4, 0.7]))
        current = {name: rng.uniform(*base_teleop.STEER_RANGE) for name in base_teleop.WHEELS}
        states, scale = kin.inverse(twist, current)
        estimate = kin.forward(
            {name: state[0] for name, state in states.items()},
            {name: state[1] for name, state in states.items()},
        )
        error = np.linalg.norm(
            np.array([estimate.vx, estimate.vy, estimate.wz])
            - scale * np.array([twist.vx, twist.vy, twist.wz]))
        max_roundtrip = max(max_roundtrip, float(error))
        feasible &= all(base_teleop.STEER_RANGE[0] <= angle <= base_teleop.STEER_RANGE[1]
                        for angle, _speed in states.values())

    fast = base_teleop.BodyTwist(8.0, -3.0, 5.0)
    saturated, saturation_scale = kin.inverse(fast)
    saturation_ok = (
        0.0 < saturation_scale < 1.0
        and max(abs(speed) for _angle, speed in saturated.values())
        <= base_teleop.WHEEL_SPEED_LIMIT[1] + 1e-12
    )
    # 100도는 이 모델의 +90도 조향 한계를 넘지만 같은 구름 방향은 바퀴 회전을 반대로
    # 한 -80도로 표현할 수 있다.
    angle, direction = limited_kin._nearest_feasible_state(0.0, np.radians(100.0))
    equivalent_ok = abs(np.degrees(angle) + 80.0) < 1e-9 and direction == -1.0
    ok = feasible and max_roundtrip < 1e-10 and saturation_ok and equivalent_ok
    print(f"Swerve kinematics gate: feasible={feasible} roundtrip={max_roundtrip:.2e} "
          f"global_saturation={saturation_ok} +/-90_equivalent={equivalent_ok}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def _brute_box_least_squares(matrix, vector, lower, upper):
    """NumPy box-QP solver 검증에 사용할 3변수 active set을 모두 열거한다."""
    best = np.inf
    for state in itertools.product((-1, 0, 1), repeat=matrix.shape[1]):
        fixed = np.array([value != 0 for value in state])
        free = ~fixed
        candidate = np.zeros(matrix.shape[1])
        for index, value in enumerate(state):
            if value < 0:
                candidate[index] = lower[index]
            elif value > 0:
                candidate[index] = upper[index]
        if np.any(free):
            residual = vector - matrix[:, fixed] @ candidate[fixed]
            candidate[free], *_ = np.linalg.lstsq(matrix[:, free], residual, rcond=None)
        if np.any(candidate < lower - 1e-9) or np.any(candidate > upper + 1e-9):
            continue
        best = min(best, float(np.linalg.norm(matrix @ candidate - vector) ** 2))
    return best


def run_box_qp_gate():
    """명시적 QP 변환과 active-set 해를 완전탐색 최적해와 비교한다."""
    rng = np.random.default_rng(20260719)
    worst_gap = 0.0
    worst_conversion_error = 0.0
    for _ in range(25):
        matrix = rng.normal(size=(8, 3))
        vector = rng.normal(size=8)
        lower = rng.uniform(-1.2, -0.1, size=3)
        upper = rng.uniform(0.1, 1.2, size=3)
        hessian, linear = bounded_optimization.least_squares_to_qp(
            matrix, vector)
        solution = bounded_optimization.bounded_quadratic_program(
            hessian, linear, lower, upper)
        qp_objective = float(
            0.5 * solution @ hessian @ solution + linear @ solution)
        objective = qp_objective + float(vector @ vector)
        optimum = _brute_box_least_squares(matrix, vector, lower, upper)
        worst_gap = max(worst_gap, objective - optimum)
        least_squares_objective = float(
            np.linalg.norm(matrix @ solution - vector) ** 2)
        worst_conversion_error = max(
            worst_conversion_error,
            abs(objective - least_squares_objective))
    ok = (worst_gap < 1e-9
          and worst_conversion_error < 1e-10)
    print(f"Box-QP gate: objective_gap={worst_gap:.2e} "
          f"conversion={worst_conversion_error:.2e}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_explicit_qp_path_gate():
    """control은 문제만 조립하고 실제 세 해법은 kinematics solver가 소유하는지 검사한다."""
    whole_body_path = (
        REPO_ROOT / "src" / "ffw_sh5_grasp" / "control" / "whole_body.py")
    optimization_path = (
        REPO_ROOT / "src" / "ffw_sh5_grasp" / "kinematics" / "optimization.py")
    whole_body_source = whole_body_path.read_text(encoding="utf-8")
    whole_body_tree = ast.parse(whole_body_source, filename=str(whole_body_path))
    optimization_tree = ast.parse(
        optimization_path.read_text(encoding="utf-8"),
        filename=str(optimization_path))
    imported_optimization = any(
        isinstance(node, ast.ImportFrom)
        and any(alias.name == "optimization" for alias in node.names)
        for node in ast.walk(whole_body_tree))
    solver_calls = {
        node.func.attr
        for node in ast.walk(whole_body_tree)
        if (isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr in {"solve", "enforce_constraints"})
    }
    solver_functions = {
        node.name for node in optimization_tree.body
        if isinstance(node, ast.FunctionDef)
    }
    required_functions = {
        "least_squares_to_qp",
        "bounded_quadratic_program",
        "bounded_quadratic_program_with_barriers",
    }
    missing = sorted(required_functions - solver_functions)
    forced_base_override = "qdot[:3] =" in whole_body_source
    ok = (not imported_optimization and not missing
          and {"solve", "enforce_constraints"} <= solver_calls
          and not forced_base_override)
    print(f"Solver ownership gate: optimization_import={imported_optimization} "
          f"missing={missing} calls={sorted(solver_calls)} "
          f"forced_base_override={forced_base_override}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_selectable_ik_methods_gate():
    """pseudoinverse, DLS, QP가 같은 공개 문제 API에서 유한한 제약 해를 내는지 검사한다."""
    matrix = np.array([[1.0, 0.0], [0.0, 2.0], [1.0, 1.0]])
    vector = np.array([0.4, -0.3, 0.1])
    lower = np.array([-0.6, -0.4])
    upper = np.array([0.7, 0.8])
    barrier_matrix = np.array([[1.0, 1.0]])
    barrier_lower = np.array([-0.2])
    solutions = {}
    for method in ("pseudoinverse", "dls", "qp"):
        solver = kinematics.DifferentialIKSolver(method, dls_damping=0.1)
        nominal = solver.solve(matrix, vector, lower, upper)
        solutions[method] = solver.enforce_constraints(
            nominal, lower, upper, barrier_matrix, barrier_lower, 1000.0)
    bounded = all(
        np.all(solution >= lower - 1e-12)
        and np.all(solution <= upper + 1e-12)
        and np.all(np.isfinite(solution))
        for solution in solutions.values())
    barriers = all(
        (barrier_matrix @ solution).item() >= -0.201
        for solution in solutions.values())
    distinct = not np.allclose(
        solutions["pseudoinverse"], solutions["dls"], atol=1e-8)
    # 첫 번째 변수를 고정한 뒤 두 번째 변수가 전체 task를 다시 맡아야 한다. 예전의
    # solve-then-clip 구현은 [0, 0.5]를 반환해 residual 절반을 그대로 남겼다.
    fixed_matrix = np.array([[1.0, 1.0]])
    fixed_vector = np.array([1.0])
    fixed_lower = np.array([0.0, -2.0])
    fixed_upper = np.array([0.0, 2.0])
    fixed_solutions = {
        method: kinematics.DifferentialIKSolver(
            method, dls_damping=0.01).solve(
                fixed_matrix, fixed_vector, fixed_lower, fixed_upper)
        for method in ("pseudoinverse", "dls")
    }
    fixed_reallocated = all(
        abs(solution[0]) < 1e-12 and solution[1] > 0.999
        for solution in fixed_solutions.values())
    rng = np.random.default_rng(20260805)
    worst_active_set_gap = 0.0
    for method in ("pseudoinverse", "dls"):
        damping = 0.08
        for _ in range(20):
            active_matrix = rng.normal(size=(8, 6))
            active_vector = rng.normal(size=8)
            active_lower = rng.uniform(-1.2, -0.05, size=6)
            active_upper = rng.uniform(0.05, 1.2, size=6)
            fixed = rng.random(6) < 0.2
            fixed_values = rng.uniform(-0.3, 0.3, size=np.count_nonzero(fixed))
            active_lower[fixed] = fixed_values
            active_upper[fixed] = fixed_values
            active_solution = kinematics.DifferentialIKSolver(
                method, dls_damping=damping).solve(
                    active_matrix, active_vector, active_lower, active_upper)
            if method == "dls":
                reference_matrix = np.vstack([
                    active_matrix, damping * np.eye(6)])
                reference_vector = np.concatenate([
                    active_vector, np.zeros(6)])
            else:
                reference_matrix, reference_vector = active_matrix, active_vector
            reference_hessian, reference_linear = (
                bounded_optimization.least_squares_to_qp(
                    reference_matrix, reference_vector))
            reference = bounded_optimization.bounded_quadratic_program(
                reference_hessian, reference_linear,
                active_lower, active_upper)
            objective = lambda value: np.linalg.norm(
                reference_matrix @ value - reference_vector) ** 2
            worst_active_set_gap = max(
                worst_active_set_gap,
                float(objective(active_solution) - objective(reference)))
    active_set_optimal = worst_active_set_gap < 1e-9
    ok = (bounded and barriers and distinct and fixed_reallocated
          and active_set_optimal)
    print(f"Selectable IK methods gate: bounded={bounded} barriers={barriers} "
          f"pinv_vs_dls_distinct={distinct} reallocated={fixed_reallocated} "
          f"active_set_gap={worst_active_set_gap:.1e}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_arm_only_selectable_methods_gate(model):
    """Whole-body OFF에서도 pseudoinverse/DLS가 고정 body 몫을 팔로 재분배하는지 검사한다."""
    data = mujoco.MjData(model)
    _reset(model, data)
    solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS,
        collision_avoidance=False)
    targets = {}
    for side in ("r", "l"):
        state = solver.site_state(data, side)
        lateral = 0.03 if side == "r" else -0.03
        targets[side] = (
            state.position + np.array([0.08, lateral, 0.04]),
            state.quaternion,
        )

    rates = {}
    body_fixed = True
    for method in ("pseudoinverse", "dls", "qp"):
        solver.set_solver_method(method)
        command = solver.solve(
            data, targets, 0.04, whole_body_enabled=False)
        qdot = command.generalized_velocity
        body_fixed &= np.linalg.norm(qdot[:4]) < 1e-12
        rates[method] = min(
            float(np.linalg.norm(solver.site_state(data, side).jacobian[:3] @ qdot))
            for side in ("r", "l"))
    comparable = (
        rates["pseudoinverse"] >= 0.90 * rates["qp"]
        and rates["dls"] >= 0.90 * rates["qp"])
    ok = body_fixed and comparable and min(rates.values()) > 0.5
    print(f"Arm-only selectable methods gate: rates={rates} body_fixed={body_fixed} "
          f"comparable={comparable}: {'OK' if ok else 'FAIL'}")
    return ok


def run_qp_velocity_normalization_gate(model):
    """모든 QP strength가 대응 속도로 무차원화되고 명령에 반영되는지 검사한다."""
    data = mujoco.MjData(model)
    _reset(model, data)

    def make_solver():
        return whole_body_ik.WholeBodyIK(
            model, {side: f"grasp_target_{side}" for side in ("r", "l")},
            ARMS, collision_avoidance=False)

    probe = make_solver()
    unit_cost = pose_tasks.normalized_weights(
        np.ones(len(probe.joint_names)), probe.velocity_limits)
    normalized = np.allclose(
        unit_cost, 1.0 / np.square(probe.velocity_limits))
    # schema 5에도 유지된 strength 기본값은 이전 raw 비용을 그대로 재현하므로 단위 통일 자체가
    # 기본 동작을 바꾸지 않는다.
    damping_cost = pose_tasks.normalized_weights(
        probe.damping_weights, probe.velocity_limits)
    posture_cost = pose_tasks.normalized_weights(
        probe.posture_weights, probe.velocity_limits)
    hand_cost = pose_tasks.normalized_weights(
        [probe.position_weight, probe.orientation_weight],
        [probe.max_task_linear_speed, probe.max_task_angular_speed])
    rigid_cost = pose_tasks.normalized_weights(
        [probe.rigid_grasp_position_weight,
         probe.rigid_grasp_orientation_weight],
        [probe.max_task_linear_speed, probe.max_task_angular_speed])
    common_base_cost = pose_tasks.normalized_weights(
        [probe.common_base_weights[0], probe.common_base_weights[2]],
        [probe.velocity_limits[0], probe.velocity_limits[2]])
    defaults_preserved = (
        np.allclose(damping_cost[[0, 2, 3, 4]], [0.25, 0.20, 0.12, 0.045])
        and np.allclose(posture_cost[[3, 4]], [0.10, 0.025])
        and np.allclose(hand_cost, [10.0, 5.0])
        and np.allclose(rigid_cost, [250.0, 250.0])
        and np.allclose(common_base_cost, [30.0, 100.0])
        and {"rigid_grasp_position", "rigid_grasp_orientation"}
        <= set(probe.qp_weights()))

    command_norms = {}
    for strength in (1e-4, 1e3):
        solver = make_solver()
        targets = {}
        for side in ("r", "l"):
            state = solver.site_state(data, side)
            targets[side] = (
                state.position + np.array([0.04, 0.0, 0.02]),
                state.quaternion,
            )
        for name in (
                "damping_base_linear", "damping_base_yaw",
                "damping_lift", "damping_arm"):
            solver.set_qp_weight(name, strength)
        command = solver.solve(data, targets, 0.04)
        command_norms[strength] = float(np.linalg.norm(
            command.generalized_velocity))
    sensitive = command_norms[1e3] < 0.25 * command_norms[1e-4]
    ok = normalized and defaults_preserved and sensitive
    print(f"QP velocity normalization gate: normalized={normalized} "
          f"defaults={defaults_preserved} command={command_norms} "
          f"sensitive={sensitive}: {'OK' if ok else 'FAIL'}")
    return ok


def run_joint_limit_cbf_gate(model):
    """Cyclo 방식 barrier 경계가 접근을 감속하고 margin 밖에서 복귀시키는지 검사한다."""
    data = mujoco.MjData(model)
    _reset(model, data)
    solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    index = solver.index["arm_r_joint4"]
    safe_high = solver.position_ranges[index, 1] - solver.joint_limit_margin
    current = data.qpos[solver.qpos_adrs].copy()
    current[index] = safe_high - 0.005
    lower, upper = solver._velocity_bounds(current, 0.04)
    expected_upper = solver.joint_limit_gain * 0.005
    slowed = 0.0 <= upper[index] <= expected_upper + 1e-12
    stays_safe = current[index] + 0.04 * upper[index] <= safe_high + 1e-12

    current[index] = safe_high + 0.01
    recovery_lower, recovery_upper = solver._velocity_bounds(current, 0.04)
    recovers = recovery_lower[index] <= recovery_upper[index] < 0.0
    ok = slowed and stays_safe and recovers
    print(f"Joint-limit CBF gate: approach_bound={upper[index]:.4f} "
          f"safe={stays_safe} recovery={recovery_upper[index]:.4f}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


SELF_COLLISION_Q = np.array([
    -2.36395827, -0.77470818, 0.09261652, -2.40170025,
    1.90799215, -0.47054658, 0.01506766,
    0.88489933, 0.57009304, -1.24600523, 0.60312349,
    0.13156373, 0.24918186, 0.61419769,
])
HAND_COLLISION_Q = np.array([
    -0.09212671, 0.0, 0.38088724, -1.46555720,
    -0.03419333, -0.02012158, -0.37858088,
    -0.09212671, 0.0, -0.38088724, -1.46555720,
    0.03419333, -0.02012158, 0.37858088,
])


def _self_collision_fixture(model):
    """자기 충돌에 가까운 고정 관절 자세와 solver를 재현 가능한 fixture로 만든다."""
    data = mujoco.MjData(model)
    _reset(model, data)
    solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    arm_indices = np.r_[solver.side_indices["r"], solver.side_indices["l"]]
    data.qpos[solver.qpos_adrs[arm_indices]] = SELF_COLLISION_Q
    mujoco.mj_forward(model, data)
    pair = next(pair for pair in solver.collision_pairs
                if pair.name == "body:hx5_r_base/lift_link")
    return data, solver, pair


def run_collision_gradient_gate(model):
    """최근접점 Jacobian이 수치 signed-distance 미분과 일치하는지 검사한다."""
    data, solver, pair = _self_collision_fixture(model)
    result = kinematics.collision_distance_gradient(
        model, data, pair, solver.kinematic_tree, solver.joint_ids, 0.10)
    step = 1e-6
    numerical = np.zeros(len(solver.dof_ids))
    for index, qpos_adr in enumerate(solver.qpos_adrs):
        distances = []
        for sign in (-1.0, 1.0):
            scratch = mujoco.MjData(model)
            scratch.qpos[:] = data.qpos
            scratch.qpos[qpos_adr] += sign * step
            mujoco.mj_forward(model, scratch)
            perturbed = kinematics.collision_distance_gradient(
                model, scratch, pair, solver.kinematic_tree,
                solver.joint_ids, 0.10)
            distances.append(perturbed.distance)
        numerical[index] = (distances[1] - distances[0]) / (2.0 * step)
    error = float(np.max(np.abs(result.gradient - numerical)))
    # 손바닥 box는 MuJoCo 3.10에서 불안정한 GJK feature 전환을 일으키므로 전용 보수적
    # sphere 근사가 같은 유한차분 시험에서 연속성을 유지해야 한다.
    arm_indices = np.r_[solver.side_indices["r"], solver.side_indices["l"]]
    data.qpos[solver.qpos_adrs[arm_indices]] = HAND_COLLISION_Q
    mujoco.mj_forward(model, data)
    hand_pair = next(pair for pair in solver.collision_pairs
                     if "hx5_r_base/hx5_l_base" in pair.name)
    hand_result = kinematics.collision_distance_gradient(
        model, data, hand_pair, solver.kinematic_tree, solver.joint_ids, 0.10)
    hand_numerical = np.zeros(len(solver.dof_ids))
    for index, qpos_adr in enumerate(solver.qpos_adrs):
        distances = []
        for sign in (-1.0, 1.0):
            scratch = mujoco.MjData(model)
            scratch.qpos[:] = data.qpos
            scratch.qpos[qpos_adr] += sign * step
            mujoco.mj_forward(model, scratch)
            perturbed = kinematics.collision_distance_gradient(
                model, scratch, hand_pair, solver.kinematic_tree,
                solver.joint_ids, 0.10)
            distances.append(perturbed.distance)
        hand_numerical[index] = (distances[1] - distances[0]) / (2.0 * step)
    hand_error = float(np.max(np.abs(
        hand_result.gradient - hand_numerical)))
    pair_scope_ok = not any(
        token in pair.name for pair in solver.collision_pairs
        for token in ("wheel", "floor", "finger", "can"))
    ok = (result.distance < solver.collision_buffer and error < 1e-6
          and hand_pair.mode == "bounding_sphere"
          and hand_result.distance < solver.collision_safe_distance
          and hand_error < 1e-6 and pair_scope_ok)
    print(f"Collision gradient gate: pairs={len(solver.collision_pairs)} "
          f"body/hand={result.distance*1000:.1f}/{hand_result.distance*1000:.1f}mm "
          f"max_error={max(error, hand_error):.2e} "
          f"scope={pair_scope_ok}: {'OK' if ok else 'FAIL'}")
    return ok


def run_self_collision_cbf_gate(model):
    """10 mm 안쪽의 내향 손 명령이 분리 속도로 바뀌는지 검사한다."""
    data = mujoco.MjData(model)
    _reset(model, data)
    probe_solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    arm_indices = np.r_[probe_solver.side_indices["r"], probe_solver.side_indices["l"]]
    data.qpos[probe_solver.qpos_adrs[arm_indices]] = HAND_COLLISION_Q
    mujoco.mj_forward(model, data)
    pair = next(pair for pair in probe_solver.collision_pairs
                if "hx5_r_base/hx5_l_base" in pair.name)
    result = kinematics.collision_distance_gradient(
        model, data, pair, probe_solver.kinematic_tree,
        probe_solver.joint_ids, 0.10)
    normal = result.point_b - result.point_a
    normal /= np.linalg.norm(normal)
    sites = _sites(model)
    targets = _target_poses(data, sites, [0.0, 0.0, 0.0])
    targets["r"] = (targets["r"][0] - 0.08 * normal, targets["r"][1])

    free_solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS,
        collision_avoidance=False)
    safe_solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    qpos_before = data.qpos.copy()
    free = free_solver.solve(data, targets, 0.04, active_sides=("r",))
    safe = safe_solver.solve(data, targets, 0.04, active_sides=("r",))
    constraints = safe_solver._collision_constraints(data, 0.04)
    constraint = next(
        item for item in constraints if item.name == pair.name)
    lower = constraint.lower
    free_rate = float(constraint.gradient @ free.generalized_velocity)
    safe_rate = float(constraint.gradient @ safe.generalized_velocity)
    ok = (np.array_equal(data.qpos, qpos_before)
          and constraint.distance < safe_solver.collision_safe_distance
          and free_rate < lower - 0.01
          and safe_rate >= lower - 1e-3
          and pair.name in safe.active_collision_pairs
          and safe.collision_constraint_violation < 1e-3)
    print(f"Self-collision CBF gate: d={constraint.distance*1000:.1f}mm "
          f"distance_rate={free_rate:+.3f}->{safe_rate:+.3f} "
          f"required>={lower:+.3f} violation={safe.collision_constraint_violation:.2e}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_table_collision_cbf_gate(model):
    """양손 하향 명령이 완화된 10 mm margin을 넘지 않는지 검사한다."""
    data = mujoco.MjData(model)
    _reset(model, data)
    probe = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    data.qpos[probe.qpos_adrs[probe.index["lift_joint"]]] -= 0.035
    mujoco.mj_forward(model, data)
    sites = _sites(model)
    targets = _target_poses(data, sites, [0.0, 0.0, -0.15])
    free_solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS,
        collision_avoidance=False)
    safe_solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    free = free_solver.solve(data, targets, 0.04)
    safe = safe_solver.solve(data, targets, 0.04)
    constraints = safe_solver._collision_constraints(data, 0.04)
    matrix = np.vstack([constraint.gradient for constraint in constraints])
    lower = np.array([constraint.lower for constraint in constraints])
    free_violation = float(np.max(lower - matrix @ free.generalized_velocity))
    safe_violation = float(np.max(np.maximum(
        lower - matrix @ safe.generalized_velocity, 0.0)))
    predicted_distances = np.array([
        constraint.distance for constraint in constraints
    ]) + 0.04 * (matrix @ safe.generalized_velocity)

    # 충돌 검사는 25 Hz UI의 프레임 예산 40 ms보다 충분히 빨라야 한다.
    for _ in range(10):
        safe_solver.solve(data, targets, 0.04)
    start = time.perf_counter()
    for _ in range(100):
        safe_solver.solve(data, targets, 0.04)
    milliseconds = 1000.0 * (time.perf_counter() - start) / 100.0
    workspace_only = all(
        constraint.name.startswith("workspace:") for constraint in constraints)
    normalized_correction = float(np.linalg.norm(
        (safe.generalized_velocity - free.generalized_velocity)
        / safe_solver.velocity_limits))
    ok = (len(constraints) >= 2 and workspace_only
          and free_violation > 0.10 and safe_violation < 1e-3
          and np.min(predicted_distances) >= safe_solver.collision_safe_distance - 1e-4
          # 정규화 projection은 lift만 강제하지 않고 속도 여유가 큰 팔과 역할을
          # 나누므로 특정 raw 관절속도 대신 전체 무차원 safety correction을 검사한다.
          and normalized_correction > 0.05
          and milliseconds < 5.0)
    print(f"Table collision CBF gate: active={len(constraints)} "
          f"violation={free_violation:.3f}->{safe_violation:.2e} "
          f"lift={free.generalized_velocity[3]:+.3f}->"
          f"{safe.generalized_velocity[3]:+.3f} correction={normalized_correction:.3f} "
          f"{milliseconds:.2f}ms: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_collision_inactive_regression_gate(model):
    """장애물에서 멀 때 충돌 회피 활성화가 출력에 영향을 주지 않는지 검사한다."""
    data = mujoco.MjData(model)
    _reset(model, data)
    sites = _sites(model)
    targets = _target_poses(data, sites, [0.0, 0.0, 0.0])
    free_solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS,
        collision_avoidance=False)
    safe_solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    free = free_solver.solve(data, targets, 0.04)
    safe = safe_solver.solve(data, targets, 0.04)
    difference = float(np.max(np.abs(
        free.generalized_velocity - safe.generalized_velocity)))
    ok = (not safe.active_collision_pairs and difference < 1e-12
          and np.isinf(safe.minimum_collision_distance))
    print(f"Inactive collision regression gate: active={len(safe.active_collision_pairs)} "
          f"max_command_delta={difference:.2e}: {'OK' if ok else 'FAIL'}")
    return ok


def run_rigid_grasp_gate(model):
    """캡처한 양손 관계가 충돌하는 독립 손 명령보다 우선하는지 검사한다."""
    data = mujoco.MjData(model)
    _reset(model, data)
    sites = _sites(model)
    targets = _target_poses(data, sites, [0.0, 0.0, 0.0])
    targets["r"] = (targets["r"][0] + np.array([0.08, 0.0, 0.0]), targets["r"][1])
    targets["l"] = (targets["l"][0] - np.array([0.08, 0.0, 0.0]), targets["l"][1])

    free_solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    rigid_solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    rigid_solver.set_rigid_grasp(data, True)
    free = free_solver.solve(data, targets, 0.04)
    rigid = rigid_solver.solve(data, targets, 0.04, rigid_grasp=True)

    states = {
        side: rigid_solver.site_state(data, side) for side in ("r", "l")
    }
    grasp_jacobian, desired_velocity = bimanual.rigid_grasp_task(
        rigid_solver._rigid_grasp_reference,
        states,
        0.04,
        rigid_solver.max_task_linear_speed,
        rigid_solver.max_task_angular_speed,
    )
    free_residual = float(np.linalg.norm(
        grasp_jacobian @ free.generalized_velocity - desired_velocity))
    rigid_residual = float(np.linalg.norm(
        grasp_jacobian @ rigid.generalized_velocity - desired_velocity))
    ok = free_residual > 0.05 and rigid_residual < 0.25 * free_residual
    print(f"Rigid-grasp gate: relative_twist={free_residual:.3f}->{rigid_residual:.3f}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_rigid_grasp_physical_gate():
    """가상 물체 MoveL이 실제 동역학에서도 캡처한 관계를 보존하는지 검사한다."""
    app = teleop_app.TeleopApp.__new__(teleop_app.TeleopApp)
    app._setup_sim()
    app.q_des = {
        "r": teleop_app.HOME_Q_R.copy(),
        "l": teleop_app.HOME_Q_L.copy(),
    }
    app.arm_mode = {"r": "ik", "l": "ik"}
    app.fk_q_deg = {
        side: np.degrees(q_des).tolist()
        for side, q_des in app.q_des.items()
    }
    app.frame_dt = 0.04
    app.steps_per_frame = round(app.frame_dt / app.model.opt.timestep)
    app.ik_err_mm = {"r": 0.0, "l": 0.0}
    no_keys = {key: False for key in ("w", "a", "s", "d", "left", "right")}

    app.capture_grasp()
    reference = app.whole_body_solver._rigid_grasp_reference
    start_states = {
        side: app.whole_body_solver.site_state(app.data, side)
        for side in ("r", "l")
    }
    start_midpoint = 0.5 * (
        start_states["r"].position + start_states["l"].position)
    app.targets["virtual_object_pos"][0] -= 0.08
    app.targets["virtual_object_rpy"][2] += 12.0
    for _ in range(50):
        app._step_physics(no_keys)

    right = app.whole_body_solver.site_state(app.data, "r")
    left = app.whole_body_solver.site_state(app.data, "l")
    right_rotation = kinematics_math.rotation_from_quaternion(right.quaternion)
    position_right = right_rotation.T @ (left.position - right.position)
    rotation_right = right_rotation.T @ kinematics_math.rotation_from_quaternion(
        left.quaternion)
    current_relative_quaternion = kinematics_math.quaternion_from_rotation(
        rotation_right)
    reference_relative_quaternion = kinematics_math.quaternion_from_rotation(
        reference["rotation_right"])
    position_drift = float(np.linalg.norm(
        position_right - reference["position_right"]))
    orientation_drift = float(np.linalg.norm(
        kinematics.shortest_orientation_error(
            reference_relative_quaternion, current_relative_quaternion)))
    final_midpoint = 0.5 * (right.position + left.position)
    moved = float(np.linalg.norm(final_midpoint - start_midpoint))
    ok = position_drift < 0.012 and orientation_drift < np.radians(3.0) and moved > 0.03
    print(f"Rigid-grasp physical gate: drift={position_drift*1000:.1f}mm/"
          f"{np.degrees(orientation_drift):.2f}deg moved={moved*1000:.1f}mm: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_world_anchor_gate():
    """베이스 이동 중 전신 목표가 월드에 고정되고 실제 손 오차가 생기는지 검사한다."""
    app = teleop_app.TeleopApp.__new__(teleop_app.TeleopApp)
    app._setup_sim()
    app.targets["pos_r"] = [0.08, -0.03, 0.04]
    app.targets["rpy_r"] = [5.0, -7.0, 11.0]
    hand_before = teleop_targets.target_world_pose(app, "r")
    virtual_before = teleop_targets.virtual_object_world_pose(app)

    base_bindings = app.bindings.base
    app.data.qpos[base_bindings.x_qpos] += 0.25
    app.data.qpos[base_bindings.y_qpos] -= 0.12
    app.data.qpos[base_bindings.yaw_qpos] += np.radians(35.0)
    mujoco.mj_forward(app.model, app.data)
    hand_after = teleop_targets.target_world_pose(app, "r")
    virtual_after = teleop_targets.virtual_object_world_pose(app)

    hand_fixed = (np.linalg.norm(hand_after[0] - hand_before[0]) < 1e-12
                  and abs(abs(np.dot(hand_after[1], hand_before[1])) - 1.0) < 1e-12)
    virtual_fixed = (np.linalg.norm(virtual_after[0] - virtual_before[0]) < 1e-12
                     and abs(abs(np.dot(virtual_after[1], virtual_before[1])) - 1.0) < 1e-12)
    actual_hand = app.whole_body_solver.site_state(app.data, "r")
    actual_hand_moved = np.linalg.norm(actual_hand.position - hand_before[0]) > 0.1
    ok = hand_fixed and virtual_fixed and actual_hand_moved
    print(f"World target anchor gate: hand_fixed={hand_fixed} virtual_fixed={virtual_fixed} "
          f"actual_robot_moved={actual_hand_moved}: {'OK' if ok else 'FAIL'}")
    return ok


def run_manual_handover_gate():
    """수동 베이스 이동이 목표를 운반하고 복귀 명령 없이 제어권을 넘기는지 검사한다."""
    app = teleop_app.TeleopApp.__new__(teleop_app.TeleopApp)
    app._setup_sim()
    targets_before = {
        side: teleop_targets.target_world_pose(app, side) for side in ("r", "l")}
    virtual_before = teleop_targets.virtual_object_world_pose(app)
    app.whole_body_solver.solve(app.data, targets_before, 0.04)

    base_bindings = app.bindings.base
    previous_base = np.array([
        app.data.qpos[base_bindings.x_qpos],
        app.data.qpos[base_bindings.y_qpos],
        app.data.qpos[base_bindings.yaw_qpos],
    ])
    app.data.qpos[base_bindings.x_qpos] += 0.25
    app.data.qpos[base_bindings.y_qpos] -= 0.10
    app.data.qpos[base_bindings.yaw_qpos] += np.radians(20.0)
    mujoco.mj_forward(app.model, app.data)
    current_base = np.array([
        app.data.qpos[base_bindings.x_qpos],
        app.data.qpos[base_bindings.y_qpos],
        app.data.qpos[base_bindings.yaw_qpos],
    ])
    teleop_targets.carry_world_targets_with_base(app, previous_base, current_base)
    targets_after = {
        side: teleop_targets.target_world_pose(app, side) for side in ("r", "l")}
    virtual_after = teleop_targets.virtual_object_world_pose(app)

    delta_yaw = current_base[2] - previous_base[2]
    c, s = np.cos(delta_yaw), np.sin(delta_yaw)
    rotation = np.array([[c, -s], [s, c]])

    def expected_position(position):
        """이전 베이스 기준 월드 점의 수동 이동 후 기대 위치를 계산한다."""
        expected = np.asarray(position).copy()
        expected[:2] = current_base[:2] + rotation @ (expected[:2] - previous_base[:2])
        return expected

    carried_positions = all(
        np.linalg.norm(targets_after[side][0] - expected_position(targets_before[side][0]))
        < 1e-12 for side in ("r", "l"))
    carried_virtual = np.linalg.norm(
        virtual_after[0] - expected_position(virtual_before[0])) < 1e-12
    delta_quat = np.array([np.cos(delta_yaw / 2), 0.0, 0.0, np.sin(delta_yaw / 2)])
    carried_orientations = True
    for side in ("r", "l"):
        expected_quat = np.zeros(4)
        mujoco.mju_mulQuat(expected_quat, delta_quat, targets_before[side][1])
        carried_orientations &= abs(abs(np.dot(expected_quat, targets_after[side][1])) - 1.0) < 1e-12

    app.whole_body_solver.rebase(app.data, targets_after)
    handover = app.whole_body_solver.solve(app.data, targets_after, 0.04)
    handover_speed = np.linalg.norm([
        handover.base_twist.vx, handover.base_twist.vy, handover.base_twist.wz])
    no_return = handover_speed < 1e-10

    shifted_targets = {
        side: (pose[0] + np.array([0.05, 0.0, 0.0]), pose[1])
        for side, pose in targets_after.items()
    }
    resumed = app.whole_body_solver.solve(app.data, shifted_targets, 0.04)
    resumed_speed = np.linalg.norm([resumed.base_twist.vx, resumed.base_twist.vy])
    wbik_resumes = resumed_speed > 0.05
    ok = (carried_positions and carried_virtual and carried_orientations
          and no_return and wbik_resumes)
    print(f"Manual handover gate: carried_pos={carried_positions} "
          f"carried_ori={carried_orientations} virtual={carried_virtual} "
          f"return_twist={handover_speed:.2e} resumed={resumed_speed:.3f}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_manual_release_physical_gate():
    """전체 경로의 입력 해제가 반동 없이 차체와 바퀴 회전을 멈추는지 검사한다."""
    app = teleop_app.TeleopApp.__new__(teleop_app.TeleopApp)
    app._setup_sim()
    app.q_des = {
        "r": teleop_app.HOME_Q_R.copy(),
        "l": teleop_app.HOME_Q_L.copy(),
    }
    app.arm_mode = {"r": "ik", "l": "ik"}
    app.fk_q_deg = {
        side: np.degrees(q_des).tolist()
        for side, q_des in app.q_des.items()
    }
    app.frame_dt = 0.04
    app.steps_per_frame = round(app.frame_dt / app.model.opt.timestep)
    app.ik_err_mm = {"r": 0.0, "l": 0.0}
    no_keys = {key: False for key in ("w", "a", "s", "d", "left", "right")}
    backward = dict(no_keys, s=True)

    for _ in range(25):
        app._step_physics(backward)
    base_bindings = app.bindings.base
    release_x = float(app.data.qpos[base_bindings.x_qpos])
    positions = [release_x]
    stop_time = None
    wheel_stop_time = None
    for frame in range(75):
        app._step_physics(no_keys)
        positions.append(float(app.data.qpos[base_bindings.x_qpos]))
        max_wheel_speed = max(
            abs(app.data.qvel[wheel.drive_dof])
            for wheel in app.bindings.wheels.values())
        if wheel_stop_time is None and max_wheel_speed < 0.01:
            wheel_stop_time = (frame + 1) * app.frame_dt
        if (stop_time is None
                and abs(app.data.qvel[base_bindings.x_dof]) < 0.01
                and not app._manual_override_active):
            stop_time = (frame + 1) * app.frame_dt

    minimum_x = min(positions)
    return_distance = positions[-1] - minimum_x
    braking_excursion = release_x - minimum_x
    ok = (return_distance < 0.005
          and stop_time is not None and stop_time < 0.5
          and wheel_stop_time is not None and wheel_stop_time < 0.5)
    print(f"Manual release physical gate: brake_excursion={braking_excursion*1000:.1f}mm "
          f"return={return_distance*1000:.3f}mm base_stop={stop_time}s "
          f"wheel_stop={wheel_stop_time}s: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_whole_body_solver_gate(model):
    """WBIK가 상태를 쓰지 않고 base·lift·양팔을 한계 안에서 함께 사용하는지 검사한다."""
    data = mujoco.MjData(model)
    _reset(model, data)
    sites = _sites(model)
    solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    targets = _target_poses(data, sites, [0.12, 0.0, 0.08])
    qpos_before = data.qpos.copy()
    lift_qadr = model.jnt_qposadr[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "lift_joint")]
    command = solver.solve(
        data, targets, 0.04,
        arm_nominal={"r": HOME, "l": HOME},
        lift_nominal=float(data.qpos[lift_qadr]),
    )
    read_only = np.array_equal(data.qpos, qpos_before)
    qdot = command.generalized_velocity
    base_used = np.linalg.norm(qdot[:3]) > 1e-3
    lift_used = abs(qdot[3]) > 1e-3
    both_arms_used = all(np.linalg.norm(qdot[solver.side_indices[s]]) > 1e-3 for s in ("r", "l"))
    within_velocity_limits = np.all(np.abs(qdot) <= solver.velocity_limits + 1e-12)

    scratch = mujoco.MjData(model)
    scratch.qpos[:] = data.qpos
    full_velocity = np.zeros(model.nv)
    full_velocity[solver.dof_ids] = qdot
    before_error = sum(np.linalg.norm(targets[s][0] - data.site_xpos[sites[s]]) for s in sites)
    mujoco.mj_integratePos(model, scratch.qpos, full_velocity, 0.04)
    mujoco.mj_forward(model, scratch)
    after_error = sum(np.linalg.norm(targets[s][0] - scratch.site_xpos[sites[s]]) for s in sites)
    descent = after_error < before_error
    ok = read_only and base_used and lift_used and both_arms_used and within_velocity_limits and descent
    print(f"Whole-body solver gate: read_only={read_only} base={base_used} lift={lift_used} "
          f"both_arms={both_arms_used} limits={within_velocity_limits} "
          f"error={before_error*1000:.1f}->{after_error*1000:.1f}mm: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_base_participation_gate(model):
    """베이스 참여율과 base-only OFF가 QP bound와 나머지 자유도를 보존하는지 검사한다."""
    data = mujoco.MjData(model)
    _reset(model, data)
    sites = _sites(model)
    targets = _target_poses(data, sites, [0.12, 0.0, 0.08])
    solver_kwargs = {
        "model": model,
        "site_names": {side: f"grasp_target_{side}" for side in ("r", "l")},
        "arm_joint_names": ARMS,
        "collision_avoidance": False,
    }
    full_solver = whole_body_ik.WholeBodyIK(**solver_kwargs)
    limited_solver = whole_body_ik.WholeBodyIK(
        **solver_kwargs, base_participation_scale=0.05)
    disabled_solver = whole_body_ik.WholeBodyIK(
        **solver_kwargs, base_participation_scale=0.0)

    full = full_solver.solve(data, targets, 0.04)
    limited = limited_solver.solve(data, targets, 0.04)
    disabled = disabled_solver.solve(data, targets, 0.04)
    limited_bound = 0.05 * limited_solver.velocity_limits[:3]
    limited_respects_bound = np.all(
        np.abs(limited.generalized_velocity[:3]) <= limited_bound + 1e-12)
    reduced = (
        np.linalg.norm(limited.generalized_velocity[:3])
        < np.linalg.norm(full.generalized_velocity[:3]))
    base_exactly_off = np.array_equal(
        disabled.generalized_velocity[:3], np.zeros(3))
    lift_and_arms_remain = np.linalg.norm(
        disabled.generalized_velocity[3:]) > 1e-3
    ok = (limited_respects_bound and reduced
          and base_exactly_off and lift_and_arms_remain)
    print(f"Base participation gate: limited_bound={limited_respects_bound} "
          f"reduced={reduced} base_off={base_exactly_off} "
          f"lift_arms_active={lift_and_arms_remain}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_arm_only_solver_gate(model):
    """OFF가 유효한 팔 IK는 유지하면서 베이스와 리프트를 완전히 막는지 검사한다."""
    data = mujoco.MjData(model)
    _reset(model, data)
    sites = _sites(model)
    solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    targets = _target_poses(data, sites, [0.04, 0.0, 0.02])
    qpos_before = data.qpos.copy()
    command = solver.solve(
        data, targets, 0.04,
        arm_nominal={"r": HOME, "l": HOME},
        lift_nominal=float(data.qpos[solver.qpos_adrs[3]] + 0.15),
        whole_body_enabled=False,
    )
    qdot = command.generalized_velocity
    body_pinned = np.array_equal(qdot[:4], np.zeros(4))
    twist_zero = (command.base_twist.vx == 0.0 and command.base_twist.vy == 0.0
                  and command.base_twist.wz == 0.0)
    both_arms_used = all(
        np.linalg.norm(qdot[solver.side_indices[side]]) > 1e-4 for side in ("r", "l"))

    scratch = mujoco.MjData(model)
    scratch.qpos[:] = data.qpos
    velocity = np.zeros(model.nv)
    velocity[solver.dof_ids] = qdot
    before_error = _pose_error_metric(data, targets, sites)
    mujoco.mj_integratePos(model, scratch.qpos, velocity, 0.04)
    mujoco.mj_forward(model, scratch)
    after_error = _pose_error_metric(scratch, targets, sites)
    read_only = np.array_equal(data.qpos, qpos_before)
    descent = after_error < before_error
    ok = body_pinned and twist_zero and both_arms_used and read_only and descent
    print(f"Arm-only solver gate: body_qdot_zero={body_pinned} twist_zero={twist_zero} "
          f"both_arms={both_arms_used} read_only={read_only} "
          f"error={before_error*1000:.1f}->{after_error*1000:.1f}mm: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_solver_latency_gate(model, solve_count=200):
    """Bounded QP가 25 Hz 앱의 프레임 예산보다 충분히 빠른지 검사한다."""
    data = mujoco.MjData(model)
    _reset(model, data)
    sites = _sites(model)
    solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    targets = _target_poses(data, sites, [0.12, 0.08, 0.04])
    for _ in range(10):
        solver.solve(data, targets, 0.04)
    start = time.perf_counter()
    for _ in range(solve_count):
        solver.solve(data, targets, 0.04)
    milliseconds = 1000.0 * (time.perf_counter() - start) / solve_count
    ok = milliseconds < 5.0
    print(f"Whole-body latency gate: {milliseconds:.3f}ms/solve (<5ms): "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_randomized_whole_body_gate(model, trial_count=40):
    """XYZ와 yaw 목표에서 한 스텝 하강, 읽기 전용과 경계를 반복 검사한다."""
    rng = np.random.default_rng(20260718)
    successes = 0
    worst_ratio = 0.0
    for _ in range(trial_count):
        data = mujoco.MjData(model)
        _reset(model, data)
        sites = _sites(model)
        solver = whole_body_ik.WholeBodyIK(
            model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
        delta = rng.uniform([-0.14, -0.14, -0.08], [0.14, 0.14, 0.12])
        yaw = rng.uniform(-np.radians(18.0), np.radians(18.0))
        targets = _target_poses(data, sites, delta)
        yaw_quat = np.array([np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)])
        for side in targets:
            position, quaternion = targets[side]
            target_quaternion = np.zeros(4)
            mujoco.mju_mulQuat(target_quaternion, yaw_quat, quaternion)
            targets[side] = (position, target_quaternion)

        qpos_before = data.qpos.copy()
        command = solver.solve(
            data, targets, 0.04,
            arm_nominal={"r": HOME, "l": HOME},
            lift_nominal=float(data.qpos[solver.qpos_adrs[3]]),
        )
        scratch = mujoco.MjData(model)
        scratch.qpos[:] = data.qpos
        velocity = np.zeros(model.nv)
        velocity[solver.dof_ids] = command.generalized_velocity
        before = _pose_error_metric(data, targets, sites)
        mujoco.mj_integratePos(model, scratch.qpos, velocity, 0.04)
        mujoco.mj_forward(model, scratch)
        after = _pose_error_metric(scratch, targets, sites)
        ratio = after / max(before, 1e-9)
        worst_ratio = max(worst_ratio, ratio)
        bounded = np.all(
            np.abs(command.generalized_velocity) <= solver.velocity_limits + 1e-12)
        if (np.array_equal(data.qpos, qpos_before) and bounded and after < before
                and np.isfinite(command.generalized_velocity).all()):
            successes += 1
    ok = successes >= trial_count - 1
    print(f"Randomized WBIK gate: {successes}/{trial_count} descent+read-only+bounded "
          f"worst_ratio={worst_ratio:.3f}: {'OK' if ok else 'FAIL'}")
    return ok


def _physical_whole_body_trial(model, delta, duration=1.5, yaw_delta=0.0,
                               rotate_positions=False):
    """양손 목표 하나를 팔·리프트와 실제 바퀴·지면 접촉으로 실행한다."""
    data = mujoco.MjData(model)
    _reset(model, data)
    sites = _sites(model)
    solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    start_targets = _target_poses(data, sites, [0.0, 0.0, 0.0])
    targets = _target_poses(data, sites, delta)
    if yaw_delta:
        center = np.mean([data.site_xpos[site] for site in sites.values()], axis=0)
        cy, sy = np.cos(yaw_delta), np.sin(yaw_delta)
        rotation = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        rotation_quat = np.array([np.cos(yaw_delta / 2.0), 0.0, 0.0,
                                  np.sin(yaw_delta / 2.0)])
        for side, site in sites.items():
            position, quaternion = targets[side]
            if rotate_positions:
                position = center + rotation @ (position - center)
            rotated_quaternion = np.zeros(4)
            mujoco.mju_mulQuat(rotated_quaternion, rotation_quat, quaternion)
            targets[side] = (position, rotated_quaternion)
    controllers = {side: arm_control.ArmTorqueController(model, ARMS[side]) for side in ("r", "l")}
    drive = base_teleop.SwerveDrive()

    steer_qadrs = {
        wheel: model.jnt_qposadr[mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"{wheel}_steer_joint")]
        for wheel in base_teleop.WHEELS
    }
    drive_dofs = {
        wheel: model.jnt_dofadr[mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"{wheel}_drive_joint")]
        for wheel in base_teleop.WHEELS
    }
    steer_aids = {wheel: mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{wheel}_steer") for wheel in base_teleop.WHEELS}
    drive_aids = {wheel: mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{wheel}_drive") for wheel in base_teleop.WHEELS}
    lift_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "lift_joint")
    lift_qadr = model.jnt_qposadr[lift_jid]
    lift_aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "lift_joint")
    base_qadrs = np.array([
        model.jnt_qposadr[mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, name)]
        for name in ("base_x", "base_y", "base_yaw")
    ])
    initial_base = data.qpos[base_qadrs].copy()
    initial_error = _pose_error_metric(data, targets, sites)
    position_ramp_frames = int(np.ceil(
        max(np.max(np.abs(targets[s][0] - start_targets[s][0])) for s in sites) / 0.03))
    orientation_ramp_frames = int(np.ceil(abs(yaw_delta) / np.radians(8.0)))
    ramp_frames = max(1, position_ramp_frames, orientation_ramp_frames)

    frame_dt = 0.04
    max_qacc = 0.0
    reach_time = None
    for frame in range(round(duration / frame_dt)):
        progress = min(1.0, (frame + 1) / ramp_frames)
        command_targets = {}
        for side in sites:
            start_pos, start_quat = start_targets[side]
            final_pos, final_quat = targets[side]
            command_pos = start_pos + progress * (final_pos - start_pos)
            command_quat = (1.0 - progress) * start_quat + progress * final_quat
            if np.dot(start_quat, final_quat) < 0.0:
                command_quat = (1.0 - progress) * start_quat - progress * final_quat
            command_quat /= np.linalg.norm(command_quat)
            command_targets[side] = (command_pos, command_quat)
        command = solver.solve(
            data, command_targets, frame_dt,
            arm_nominal={"r": HOME, "l": HOME},
            lift_nominal=float(data.qpos[lift_qadr]),
        )
        steering = {wheel: float(data.qpos[qadr]) for wheel, qadr in steer_qadrs.items()}
        wheel_velocity = {wheel: float(data.qvel[dof]) for wheel, dof in drive_dofs.items()}
        wheel_commands = drive.update_twist(
            command.base_twist, frame_dt, steering, wheel_velocity)
        for _ in range(round(frame_dt / model.opt.timestep)):
            for side in ("r", "l"):
                controllers[side].apply(data, command.arm_positions[side])
            data.ctrl[lift_aid] = command.lift_position
            for wheel, (angle, speed) in wheel_commands.items():
                data.ctrl[steer_aids[wheel]] = angle
                data.ctrl[drive_aids[wheel]] = speed
            grasp.apply_grasp(model, data, 0.0, 0.0, side="r")
            grasp.apply_grasp(model, data, 0.0, 0.0, side="l")
            mujoco.mj_step(model, data)
            max_qacc = max(max_qacc, float(np.max(np.abs(data.qacc))))
        current_error = _pose_error_metric(data, targets, sites)
        if reach_time is None and current_error < max(0.02, 0.10 * initial_error):
            reach_time = (frame + 1) * frame_dt

    final_error = _pose_error_metric(data, targets, sites)
    final_position_error = sum(
        np.linalg.norm(targets[s][0] - data.site_xpos[sites[s]]) for s in sites)
    final_orientation_error = 0.0
    for side, site in sites.items():
        current_quat = np.zeros(4)
        mujoco.mju_mat2Quat(current_quat, data.site_xmat[site])
        orientation_error = np.zeros(3)
        mujoco.mju_subQuat(orientation_error, targets[side][1], current_quat)
        final_orientation_error += np.linalg.norm(orientation_error)
    return {
        "initial_error": float(initial_error),
        "final_error": float(final_error),
        "final_position_error": float(final_position_error),
        "final_orientation_error": float(final_orientation_error),
        "base_delta": data.qpos[base_qadrs].copy() - initial_base,
        "reach_time": reach_time,
        "max_qacc": max_qacc,
        "finite": bool(np.isfinite(data.qpos).all()),
    }


def run_physical_whole_body_gate(model):
    """일반적인 전후·좌우·수직 이동과 yaw 전신 목표를 검사한다."""
    trials = {
        "longitudinal": _physical_whole_body_trial(
            model, [-0.25, 0.0, 0.08], duration=2.0),
        "lateral": _physical_whole_body_trial(model, [0.0, 0.22, 0.0], duration=2.0),
        "vertical": _physical_whole_body_trial(model, [0.0, 0.0, 0.18]),
        "yaw": _physical_whole_body_trial(
            model, [0.0, 0.0, 0.0], duration=2.0, yaw_delta=np.radians(25.0)),
    }
    ok = True
    for name, result in trials.items():
        ratio = result["final_error"] / max(result["initial_error"], 1e-9)
        stable = result["finite"] and result["max_qacc"] < 1e5
        tracked = ratio < (0.30 if name == "yaw" else 0.18)
        recruited_expected_base = {
            "longitudinal": result["base_delta"][0] < -0.12,
            "lateral": result["base_delta"][1] > 0.08,
            "vertical": True,
            "yaw": abs(result["base_delta"][2]) > 0.10,
        }[name]
        trial_ok = stable and tracked and recruited_expected_base
        ok &= trial_ok
        base = result["base_delta"]
        print(f"Physical WBIK {name}: error={result['initial_error']*1000:.1f}->"
              f"{result['final_error']*1000:.1f}mm ratio={ratio:.3f} "
              f"(pos={result['final_position_error']*1000:.1f}mm "
              f"ori={np.degrees(result['final_orientation_error']):.1f}deg) "
              f"base=({base[0]:+.3f},{base[1]:+.3f},{np.degrees(base[2]):+.1f}deg) "
              f"reach={result['reach_time']}s max|qacc|={result['max_qacc']:.0f}: "
              f"{'OK' if trial_ok else 'FAIL'}")
    return ok


def main():
    """스워브·box-QP·CBF·양손·WBIK 수치 및 물리 통합 회귀를 실행한다."""
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    ok = (run_ros_free_dependency_gate()
          and run_tree_kinematics_dependency_gate()
          and run_shared_pose_task_gate()
          and run_swerve_kinematics_gate()
          and run_box_qp_gate()
          and run_explicit_qp_path_gate()
          and run_selectable_ik_methods_gate()
          and run_arm_only_selectable_methods_gate(model)
          and run_qp_velocity_normalization_gate(model)
          and run_joint_limit_cbf_gate(model)
          and run_collision_gradient_gate(model)
          and run_self_collision_cbf_gate(model)
          and run_table_collision_cbf_gate(model)
          and run_collision_inactive_regression_gate(model)
          and run_rigid_grasp_gate(model)
          and run_rigid_grasp_physical_gate()
          and run_world_anchor_gate()
          and run_manual_handover_gate()
          and run_manual_release_physical_gate()
          and run_whole_body_solver_gate(model)
          and run_base_participation_gate(model)
          and run_arm_only_solver_gate(model)
          and run_solver_latency_gate(model)
          and run_randomized_whole_body_gate(model)
          and run_physical_whole_body_gate(model))
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
