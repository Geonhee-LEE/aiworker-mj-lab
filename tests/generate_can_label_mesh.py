"""캔 label mesh를 한 번 생성하는 asset 제작 스크립트.

제공된 ``assets/soda_can/soda_can.stl``에서 ``can_side_detail.obj``와
``can_cap_detail.obj``를 만든다. 원본은 단순 원통이 아니라 테두리, pull tab, 약간
볼록한 윗면과 오목한 바닥이 있는 상세 mesh다. label texture를 적용할 수 있도록
정점별 UV 좌표를 명시한다. STL에는 UV 정보가 없으므로 수작업 대신 기하로 다시
계산한다.

STL 정점 반지름을 높이에 따라 분석하면 전체 높이 중 약 ``z_frac=0.03``부터 0.94까지
거의 완전한 일정 반지름 원통이다. 약 1만 5천 개 정점 대부분은 복잡한 위쪽 5%와
아래쪽 3%에 몰려 있다. 따라서 면 법선 휴리스틱 없이도 label 영역을 기하 자체에서
구분할 수 있다. 느슨한 기준에서도 면의 약 78%가 수직에 가까운 법선을 가져 법선
방식은 신뢰하기 어려웠다. 옆면은 원통형 UV로 감고, 범위 밖 뚜껑 면은 texture가 없는
평면 재질이므로 단순 UV를 사용한다.

불규칙 삼각분할에는 미리 복제할 고정 seam 정점 집합이 없으므로 옆면 UV는 정점 공유
방식이 아니라 면의 각 꼭짓점마다 계산한다. 각 삼각형의 세 각도가 서로 π 이내가
되도록 2π를 더하거나 빼서 개별적으로 펼친 뒤 u로 변환한다. 이렇게 하면 삼각형이
±π seam을 가로지르는지와 관계없이 올바르게 처리할 수 있다.

실행: ``python3 tests/generate_can_label_mesh.py``
"""

import pathlib

import numpy as np
import trimesh

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
STL_PATH = REPO_ROOT / "assets" / "soda_can" / "soda_can.stl"
SIDE_OUT = REPO_ROOT / "assets" / "soda_can" / "can_side_detail.obj"
CAP_OUT = REPO_ROOT / "assets" / "soda_can" / "can_cap_detail.obj"

# mesh 자체 z 범위의 비율이다. 분석 결과 일정 반지름 구간은 약 0.03~0.94지만,
# 일부만 굽은 전환 면을 옆면으로 잘못 분류하지 않도록 측정 구간보다 안쪽에 여유를 둔다.
SIDE_Z_FRAC_LO = 0.05
SIDE_Z_FRAC_HI = 0.92


def write_obj(path, verts, faces, uvs_per_face_corner=None):
    """정점·삼각형 면과 선택적 면 꼭짓점 UV를 OBJ 형식으로 기록한다."""
    lines = [f"# tests/generate_can_label_mesh.py가 생성한 파일이므로 직접 수정하지 않는다.\n"]
    for v in verts:
        lines.append(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
    if uvs_per_face_corner is not None:
        vt_idx = 1
        for face_uvs in uvs_per_face_corner:
            for u, v in face_uvs:
                lines.append(f"vt {u:.6f} {v:.6f}\n")
        for fi, face in enumerate(faces):
            a, b, c = face + 1  # OBJ 인덱스는 1부터 시작한다.
            base = fi * 3
            lines.append(f"f {a}/{base+1} {b}/{base+2} {c}/{base+3}\n")
    else:
        for face in faces:
            a, b, c = face + 1
            lines.append(f"f {a} {b} {c}\n")
    path.write_text("".join(lines))


def main():
    """캔 STL을 라벨 옆면과 뚜껑 mesh로 분리해 두 OBJ asset을 생성한다."""
    mesh = trimesh.load(STL_PATH)
    verts = mesh.vertices
    faces = mesh.faces
    z = verts[:, 2]
    z_lo_all, z_hi_all = z.min(), z.max()
    z_lo = z_lo_all + SIDE_Z_FRAC_LO * (z_hi_all - z_lo_all)
    z_hi = z_lo_all + SIDE_Z_FRAC_HI * (z_hi_all - z_lo_all)

    face_z = z[faces].mean(axis=1)
    is_side = (face_z >= z_lo) & (face_z <= z_hi)
    side_faces = faces[is_side]
    cap_faces = faces[~is_side]
    print(f"side faces: {len(side_faces)}  cap faces: {len(cap_faces)}  "
          f"(of {len(faces)} total)")

    # 옆면 mesh는 면 꼭짓점마다 원통형 UV를 만들고 각 면의 각도를 독립적으로 펼친다.
    side_uvs = []
    for face in side_faces:
        pts = verts[face]
        angles = np.arctan2(pts[:, 1], pts[:, 0])
        for i in (1, 2):
            diff = angles[i] - angles[0]
            if diff > np.pi:
                angles[i] -= 2 * np.pi
            elif diff < -np.pi:
                angles[i] += 2 * np.pi
        us = angles / (2 * np.pi) + 0.5
        vs = np.clip((pts[:, 2] - z_lo) / (z_hi - z_lo), 0.0, 1.0)
        side_uvs.append(list(zip(us, vs)))
    write_obj(SIDE_OUT, verts, side_faces, side_uvs)
    print(f"wrote {SIDE_OUT}")

    # 뚜껑 mesh는 texture 없는 평면 재질을 쓰므로 UV가 필요 없다. 따라서 vt나
    # 슬래시가 포함된 f 항목 없이 위치만 든 OBJ가 더 단순하고 충분하다.
    write_obj(CAP_OUT, verts, cap_faces, uvs_per_face_corner=None)
    print(f"wrote {CAP_OUT}")


if __name__ == "__main__":
    main()
