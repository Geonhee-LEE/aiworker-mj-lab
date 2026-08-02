"""컴파일된 MJCF에서 복사한 불변 기구학 트리와 FK/Jacobian 계산.

MuJoCo는 include와 default가 반영된 모델 구조를 제공하는 데만 사용한다. 트리를 만든
뒤의 FK는 ``MjData``나 ``mj_forward`` 없이 NumPy 배열만으로 계산한다.
"""

from dataclasses import dataclass

import mujoco
import numpy as np

from kinematics_math import (
    axis_rotation,
    quaternion_from_rotation,
    rotation_from_quaternion,
)

_HINGE = int(mujoco.mjtJoint.mjJNT_HINGE)
_SLIDE = int(mujoco.mjtJoint.mjJNT_SLIDE)
_JOINT_KIND_NAMES = {
    int(mujoco.mjtJoint.mjJNT_FREE): "free",
    int(mujoco.mjtJoint.mjJNT_BALL): "ball",
    _SLIDE: "slide",
    _HINGE: "hinge",
}


@dataclass(frozen=True)
class SiteKinematics:
    """site의 world pose와 world-frame 6×N geometric Jacobian."""

    position: np.ndarray
    quaternion: np.ndarray
    jacobian: np.ndarray


@dataclass(frozen=True)
class KinematicJoint:
    """FK에 필요한 scalar joint 정보."""

    id: int
    name: str
    kind: int
    kind_name: str
    qpos_adr: int
    dof_adr: int
    position: np.ndarray
    axis: np.ndarray
    limited: bool
    range: np.ndarray


@dataclass(frozen=True)
class KinematicBody:
    """부모 기준 고정 변환과 소속 joint를 가진 body 노드."""

    id: int
    name: str
    parent_id: int
    position: np.ndarray
    rotation: np.ndarray
    joint_ids: tuple


@dataclass(frozen=True)
class KinematicSite:
    """body에 고정된 site 변환."""

    id: int
    name: str
    body_id: int
    position: np.ndarray
    rotation: np.ndarray


class KinematicTree:
    """MJCF body-joint-site 계층을 복사한 읽기 전용 기구학 트리."""

    def __init__(self, model):
        self.nq = int(model.nq)
        self.qpos0 = np.asarray(model.qpos0, dtype=float).copy()
        self.bodies = tuple(
            self._copy_body(model, body_id) for body_id in range(model.nbody))
        self.joints = tuple(
            self._copy_joint(model, joint_id) for joint_id in range(model.njnt))
        self.sites = tuple(
            self._copy_site(model, site_id) for site_id in range(model.nsite))
        self.joint_by_name = {
            joint.name: joint for joint in self.joints if joint.name}
        self.site_by_name = {site.name: site for site in self.sites if site.name}

        # 부모 탐색과 고정 회전 변환은 상태와 무관하므로 초기화 때 한 번만 계산한다.
        self.body_paths = tuple(self._body_path(body.id) for body in self.bodies)
        children_by_body = [[] for _ in self.bodies]
        for body in self.bodies[1:]:
            children_by_body[body.parent_id].append(body.id)
        sites_by_body = [[] for _ in self.bodies]
        for site in self.sites:
            sites_by_body[site.body_id].append(site.id)
        self.children_by_body = tuple(tuple(ids) for ids in children_by_body)
        self.sites_by_body = tuple(tuple(ids) for ids in sites_by_body)
        self.site_paths = {
            site.id: self.body_paths[site.body_id] for site in self.sites}

    @staticmethod
    def _name(model, object_type, object_id):
        return mujoco.mj_id2name(model, object_type, object_id) or ""

    @classmethod
    def _copy_body(cls, model, body_id):
        joint_address = int(model.body_jntadr[body_id])
        joint_count = int(model.body_jntnum[body_id])
        joint_ids = (() if joint_count == 0 else
                     tuple(range(joint_address, joint_address + joint_count)))
        return KinematicBody(
            id=body_id,
            name=cls._name(model, mujoco.mjtObj.mjOBJ_BODY, body_id),
            parent_id=int(model.body_parentid[body_id]),
            position=np.asarray(model.body_pos[body_id], dtype=float).copy(),
            rotation=rotation_from_quaternion(model.body_quat[body_id]),
            joint_ids=joint_ids,
        )

    @classmethod
    def _copy_joint(cls, model, joint_id):
        kind = int(model.jnt_type[joint_id])
        return KinematicJoint(
            id=joint_id,
            name=cls._name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id),
            kind=kind,
            kind_name=_JOINT_KIND_NAMES[kind],
            qpos_adr=int(model.jnt_qposadr[joint_id]),
            dof_adr=int(model.jnt_dofadr[joint_id]),
            position=np.asarray(model.jnt_pos[joint_id], dtype=float).copy(),
            axis=np.asarray(model.jnt_axis[joint_id], dtype=float).copy(),
            limited=bool(model.jnt_limited[joint_id]),
            range=np.asarray(model.jnt_range[joint_id], dtype=float).copy(),
        )

    @classmethod
    def _copy_site(cls, model, site_id):
        return KinematicSite(
            id=site_id,
            name=cls._name(model, mujoco.mjtObj.mjOBJ_SITE, site_id),
            body_id=int(model.site_bodyid[site_id]),
            position=np.asarray(model.site_pos[site_id], dtype=float).copy(),
            rotation=rotation_from_quaternion(model.site_quat[site_id]),
        )

    def _body_path(self, body_id):
        path = []
        while body_id != 0:
            path.append(body_id)
            body_id = self.bodies[body_id].parent_id
        path.reverse()
        return tuple(path)

    def _forward_body(self, qpos, body_id):
        """root에서 지정 body까지 전파하고 각 joint의 world frame을 반환한다."""
        position = np.zeros(3)
        rotation = np.eye(3)
        joint_frames = {}
        for path_body_id in self.body_paths[body_id]:
            body = self.bodies[path_body_id]
            position = position + rotation @ body.position
            rotation = rotation @ body.rotation
            for joint_id in body.joint_ids:
                joint = self.joints[joint_id]
                axis_world = rotation @ joint.axis
                anchor_world = position + rotation @ joint.position
                joint_frames[joint_id] = (joint.kind, axis_world, anchor_world)
                displacement = qpos[joint.qpos_adr] - self.qpos0[joint.qpos_adr]
                if joint.kind == _SLIDE:
                    position = position + axis_world * displacement
                elif joint.kind == _HINGE:
                    rotation = rotation @ axis_rotation(joint.axis, displacement)
                    position = anchor_world - rotation @ joint.position
                else:
                    raise NotImplementedError(
                        f"body path contains unsupported joint {joint.name!r}; "
                        "tree kinematics supports scalar hinge and slide joints")
        return position, rotation, joint_frames

    @staticmethod
    def _point_jacobian_from_frames(point_world, joint_ids, joint_frames):
        """이미 계산한 joint frame으로 world point의 3×N Jacobian을 만든다."""
        point_world = np.asarray(point_world, dtype=float)
        jacobian = np.zeros((3, len(joint_ids)))
        for column, joint_id in enumerate(joint_ids):
            frame = joint_frames.get(int(joint_id))
            if frame is None:
                continue
            kind, axis_world, anchor_world = frame
            if kind == _SLIDE:
                jacobian[:, column] = axis_world
            elif kind == _HINGE:
                jacobian[:, column] = np.cross(
                    axis_world, point_world - anchor_world)
        return jacobian

    def point_jacobian(self, qpos, body_id, point_world, joint_ids, frame_cache=None):
        """body에 고정된 world point의 3×N Jacobian을 반환한다."""
        qpos = np.asarray(qpos, dtype=float)
        if qpos.shape != (self.nq,):
            raise ValueError(f"expected qpos shape ({self.nq},), got {qpos.shape}")
        if body_id < 0 or body_id >= len(self.bodies):
            raise ValueError(f"invalid body id: {body_id}")
        if frame_cache is None:
            _, _, joint_frames = self._forward_body(qpos, body_id)
        else:
            joint_frames = frame_cache.get(body_id)
            if joint_frames is None:
                _, _, joint_frames = self._forward_body(qpos, body_id)
                frame_cache[body_id] = joint_frames
        return self._point_jacobian_from_frames(
            point_world, joint_ids, joint_frames)

    def forward_site(self, qpos, site_id, joint_ids):
        """site world pose와 선택된 joint 열의 6×N Jacobian을 반환한다."""
        qpos = np.asarray(qpos, dtype=float)
        if qpos.shape != (self.nq,):
            raise ValueError(f"expected qpos shape ({self.nq},), got {qpos.shape}")
        if site_id < 0 or site_id >= len(self.sites):
            raise ValueError(f"invalid site id: {site_id}")

        site = self.sites[site_id]
        position, rotation, joint_frames = self._forward_body(qpos, site.body_id)
        site_position = position + rotation @ site.position
        site_rotation = rotation @ site.rotation
        jacobian = np.zeros((6, len(joint_ids)))
        jacobian[:3] = self._point_jacobian_from_frames(
            site_position, joint_ids, joint_frames)
        for column, joint_id in enumerate(joint_ids):
            frame = joint_frames.get(int(joint_id))
            if frame is not None and frame[0] == _HINGE:
                jacobian[3:, column] = frame[1]
        return SiteKinematics(
            position=site_position,
            quaternion=quaternion_from_rotation(site_rotation),
            jacobian=jacobian,
        )
