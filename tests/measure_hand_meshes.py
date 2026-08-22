"""Phase 1에서 한 번 사용하는 HX5-D20 오른손 mesh 측정 스크립트.

서로 다른 손가락 STL의 AABB를 mesh 로컬 좌표계에서 구한다. MJCF가 이 mesh에 회전
없이 scale만 적용하므로 MJCF ``<mesh>`` 로컬 좌표계와 같다. 측정값은
``models/hand_only.xml``의 capsule 충돌 형상을 만드는 데 사용했다.

실행: ``python3 tests/measure_hand_meshes.py``
"""

import pathlib

import trimesh

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
MESH_DIR = REPO_ROOT / "assets" / "robotis_ffw" / "assets" / "hx5_d20" / "hx5_d20_right"
SCALE = 0.001  # MJCF의 ``<mesh scale="0.001 0.001 0.001">``와 일치한다.

MESHES = [
    "hx5_d20_base_unit.stl",
    "hx5_d20_thumb_mcp.stl",
    "hx5_d20_thumb_mcp2.stl",
    "hx5_d20_thumb_ip.stl",
    "hx5_d20_thumb_tip.stl",
    "hx5_d20_finger_mcp.stl",
    "hx5_d20_finger_pip.stl",
    "hx5_d20_finger_dip.stl",
    "hx5_d20_finger_tip.stl",
]


def main():
    """각 손 mesh의 로컬 AABB, 크기와 중심을 미터 단위로 측정해 출력한다."""
    for fname in MESHES:
        path = MESH_DIR / fname
        mesh = trimesh.load(path, force="mesh")
        verts = mesh.vertices * SCALE
        mn = verts.min(axis=0)
        mx = verts.max(axis=0)
        extent = mx - mn
        center = (mn + mx) / 2
        print(f"{fname}")
        print(f"  min={mn.round(5).tolist()}  max={mx.round(5).tolist()}")
        print(f"  extent={extent.round(5).tolist()}  center={center.round(5).tolist()}")


if __name__ == "__main__":
    main()
