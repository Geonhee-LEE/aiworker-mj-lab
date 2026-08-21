"""Record the MuJoCo robot visual scene as time-varying Rerun entities."""

import mujoco
import numpy as np


ROBOT_ROOT_BODY = "base_link"
MAX_RERUN_COLOR_INTENSITY = 0.99


class MujocoRobotRerunLogger:
    """Log visual geoms once and update their world poses for every frame."""

    def __init__(self, recording, model, data, *, root="robot"):
        self.recording = recording
        self.model = model
        self.data = data
        self.root = root
        root_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, ROBOT_ROOT_BODY)
        if root_id < 0:
            raise ValueError(f"MuJoCo model has no {ROBOT_ROOT_BODY!r} body")
        robot_body_ids = {
            body_id for body_id in range(model.nbody)
            if self._is_descendant(body_id, root_id)
        }
        self.geom_ids = np.array([
            geom_id for geom_id in range(model.ngeom)
            if int(model.geom_bodyid[geom_id]) in robot_body_ids
            and model.geom_rgba[geom_id, 3] > 0
            and model.geom_contype[geom_id] == 0
            and model.geom_conaffinity[geom_id] == 0
        ], dtype=int)

    def _is_descendant(self, body_id, root_id):
        while body_id > 0:
            if body_id == root_id:
                return True
            body_id = int(self.model.body_parentid[body_id])
        return body_id == root_id

    def _path(self, geom_id):
        name = mujoco.mj_id2name(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id))
        return f"{self.root}/geom_{geom_id}_{name or 'visual'}"

    def _color(self, geom_id):
        material_id = int(self.model.geom_matid[geom_id])
        rgba = (self.model.mat_rgba[material_id]
            if material_id >= 0 else self.model.geom_rgba[geom_id])
        display_rgba = np.array(rgba, copy=True)
        display_rgba[:3] = np.minimum(
            display_rgba[:3], MAX_RERUN_COLOR_INTENSITY)
        return np.round(display_rgba * 255).astype(np.uint8)

    def log_geometry(self):
        rr = __import__("rerun")
        for geom_id in self.geom_ids:
            geom_type = int(self.model.geom_type[geom_id])
            path = self._path(geom_id)
            color = self._color(geom_id)
            size = self.model.geom_size[geom_id]
            if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
                mesh_id = int(self.model.geom_dataid[geom_id])
                vertex_start = self.model.mesh_vertadr[mesh_id]
                vertex_count = self.model.mesh_vertnum[mesh_id]
                vertices = self.model.mesh_vert[
                    vertex_start:vertex_start + vertex_count]
                normals = self.model.mesh_normal[
                    vertex_start:vertex_start + vertex_count]
                faces = self.model.mesh_face[
                    self.model.mesh_faceadr[mesh_id]:self.model.mesh_faceadr[mesh_id]
                    + self.model.mesh_facenum[mesh_id]]
                self.recording.log(
                    path, rr.Mesh3D(
                        vertex_positions=vertices,
                        vertex_normals=normals,
                        triangle_indices=faces,
                        albedo_factor=color))
            elif geom_type == mujoco.mjtGeom.mjGEOM_BOX:
                self.recording.log(path, rr.Boxes3D(sizes=2.0 * size, colors=color))
            elif geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
                self.recording.log(
                    path, rr.Ellipsoids3D(
                        radii=float(size[0]), colors=color))
            elif geom_type in (mujoco.mjtGeom.mjGEOM_CAPSULE,
                               mujoco.mjtGeom.mjGEOM_CYLINDER):
                self.recording.log(
                    path, rr.Cylinders3D(
                        lengths=2.0 * float(size[1]), radii=float(size[0]),
                        colors=color))

    def log_poses(self):
        rr = __import__("rerun")
        for geom_id in self.geom_ids:
            self.recording.log(
                self._path(geom_id),
                rr.Transform3D(
                    translation=self.data.geom_xpos[geom_id],
                    mat3x3=self.data.geom_xmat[geom_id].reshape(3, 3)))


__all__ = ["MujocoRobotRerunLogger", "ROBOT_ROOT_BODY"]