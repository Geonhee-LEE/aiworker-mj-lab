import pathlib
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.data.paths import resolve_episode_path  # noqa: E402


def test_resolve_episode_path_prefers_explicit_path():
    assert resolve_episode_path(
        "custom.hdf5", "ignored", 12) == pathlib.Path("custom.hdf5")


def test_resolve_episode_path_builds_canonical_name():
    assert resolve_episode_path(
        None, "datasets/can_to_box", 7) == pathlib.Path(
            "datasets/can_to_box/episode_000007.hdf5")


def test_resolve_episode_path_requires_a_source():
    try:
        resolve_episode_path()
    except ValueError as error:
        assert "--episode or --dataset-dir" in str(error)
    else:
        raise AssertionError("missing episode source must be rejected")


if __name__ == "__main__":
    test_resolve_episode_path_prefers_explicit_path()
    test_resolve_episode_path_builds_canonical_name()
    test_resolve_episode_path_requires_a_source()
    print("PASS")
