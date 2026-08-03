"""컴파일된 MJCF에서 복사한 불변 기구학 트리와 FK/Jacobian 계산.

MuJoCo는 include와 default가 반영된 모델 구조를 제공하는 데만 사용한다. 트리를 만든
뒤의 FK는 ``MjData``나 ``mj_forward`` 없이 NumPy 배열만으로 계산한다.
"""

from dataclasses import dataclass

import mujoco
import numpy as np

from .rotations import (
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


# dataclass(frozen=True)는 KinematicBody, KinematicSite, SiteKinematics,
# KinematicJoint 클래스의 인스턴스를 만든 뒤 속성을 변경할 수 없게 한다.
# 이는 기구학 트리에서 각 body, site, joint의 정보가 변경되지 않도록 보장하기 위해 사용된다.
@dataclass(frozen=True)
class SiteKinematics:
    """site의 world pose와 world-frame 6×N geometric Jacobian.
    여기서 site는 로봇의 특정 지점이나 좌표계를 나타내며, world pose는 site의 위치와 회전을 나타낸다.
    geometric Jacobian은 site의 위치와 회전 변화에 대한 관절 속도의 관계를 나타내는 행렬이다.
    position: site의 world 좌표계에서의 위치를 나타내는 3차원 벡터.
    quaternion: site의 world 좌표계에서의 회전을 나타내는 4차원 쿼터니언.
    jacobian: site의 위치와 회전 변화에 대한 관절 속도의 관계를 나타내는 6×N 행렬. 상위 3행은 위치 변화에 대한 Jacobian이고, 하위 3행은 회전 변화에 대한 Jacobian이다.
    N은 고려되는 관절의 수를 나타낸다.
    """

    position: np.ndarray
    quaternion: np.ndarray
    jacobian: np.ndarray


@dataclass(frozen=True)
class KinematicJoint:
    """FK에 필요한 scalar joint 정보.
    id: joint의 고유 식별자.
    name: joint의 이름.
    kind: joint의 종류를 나타내는 정수 값. (예: hinge, slide 등)
    kind_name: joint의 종류를 나타내는 문자열.(예: "hinge", "slide" 등)
    qpos_adr: joint의 위치 변수(qpos)가 저장된 배열에서의 시작 인덱스.
    dof_adr: joint의 자유도(dof)가 저장된 배열에서의 시작 인덱스.
    position: joint의 위치를 나타내는 3차원 벡터. (joint의 anchor point)
    axis: joint의 회전 축을 나타내는 3차원 벡터. (joint의 회전 방향)
    limited: joint의 회전 범위 제한 여부를 나타내는 불리언 값.
    range: joint의 회전 범위를 나타내는 2차원 벡터. (최소값, 최대값)

    즉, KinematicJoint 클래스는 로봇의 관절에 대한 정보를 담고 있으며,
    FK 계산에 필요한 joint의 위치, 회전 축, 자유도, 제한 여부 등을 포함하고 있다.

    """
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
    """부모 기준 고정 변환과 소속 joint를 가진 body 노드.
    body는 로봇의 링크를 나타내며, 부모 body에 대한 고정 변환과 소속된 joint 정보를 포함한다.
    id: body의 고유 식별자.
    name: body의 이름.
    parent_id: 부모 body의 고유 식별자. (루트 body의 경우 0)
    position: 부모 body 기준의 위치를 나타내는 3차원 벡터. (body의 anchor point)
    rotation: 부모 body 기준의 회전을 나타내는 3×3 회전 행렬. (body의 orientation)
    joint_ids: body에 소속된 joint들의 고유 식별자 목록. (body에 연결된 joint들의 ID)

    즉, KinematicBody 클래스는 로봇의 링크에 대한 정보를 담고 있으며, 부모 body에 대한 위치와 회전, 소속된 joint들의 정보를 포함하고 있다.
    """

    id: int
    name: str
    parent_id: int
    position: np.ndarray
    rotation: np.ndarray
    joint_ids: tuple


@dataclass(frozen=True)
class KinematicSite:
    """body에 고정된 site 변환.
    id: site의 고유 식별자.
    name: site의 이름.
    body_id: site가 소속된 body의 고유 식별자.
    position: body 기준의 위치를 나타내는 3차원 벡터.
    rotation: body 기준의 회전을 나타내는 3×3 회전 행렬.
    즉, KinematicSite 클래스는 로봇의 특정 지점이나 좌표계를 나타내며, 소속된 body에 대한 위치와 회전 정보를 포함하고 있다.

    """

    id: int
    name: str
    body_id: int
    position: np.ndarray
    rotation: np.ndarray


class KinematicTree:
    """MJCF body-joint-site 계층을 복사한 읽기 전용 기구학 트리."""

    def __init__(self, model):
        """컴파일된 MuJoCo 모델에서 FK에 필요한 불변 정보만 복사한다.

        ``model``의 body, joint, site와 기본 qpos를 NumPy 기반 자료구조로 옮기고,
        각 body/site까지의 부모 경로를 미리 계산한다. 초기화 이후 FK 계산은 원본
        MuJoCo 모델이나 ``MjData``에 의존하지 않는다.
        """

        """
        nq = int(model.nq)  # 모델의 자유도 수를 가져온다. ex) 7
        qpos0 = np.asarray(model.qpos0, dtype=float).copy()  # 모델의 초기 관절 위치를 NumPy 배열로 복사한다.
        bodies = tuple(
            self._copy_body(model, body_id) for body_id in range(model.nbody))
        # 모델의 모든 body를 순회하며 KinematicBody 객체로 복사한다.
        joints = tuple(
            self._copy_joint(model, joint_id) for joint_id in range(model.njnt))
        # 모델의 모든 joint를 순회하며 KinematicJoint 객체로 복사한다.
        sites = tuple(
            self._copy_site(model, site_id) for site_id in range(model.nsite))
        # 모델의 모든 site를 순회하며 KinematicSite 객체로 복사한다.
        joint_by_name = {
            joint.name: joint for joint in self.joints if joint.name}
        # joint 이름을 키로, KinematicJoint 객체를 값으로 하는 딕셔너리를 생성한다. 이름이 없는 joint는 제외한다.
        site_by_name = {site.name: site for site in self.sites if site.name}
        # site 이름을 키로, KinematicSite 객체를 값으로 하는 딕셔너리를 생성한다. 이름이 없는 site는 제외한다.
        """
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
        """MuJoCo 객체 ID를 이름으로 바꾸며 이름이 없으면 빈 문자열을 반환한다."""
        return mujoco.mj_id2name(model, object_type, object_id) or ""

    @classmethod
    def _copy_body(cls, model, body_id):
        """MuJoCo body 하나의 부모 변환과 소속 joint ID를 불변 노드로 복사한다."""
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
        """MuJoCo joint 하나의 형식·주소·축·제한 정보를 불변 노드로 복사한다."""
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
        """MuJoCo site 하나의 소속 body와 고정 변환을 불변 노드로 복사한다."""
        return KinematicSite(
            id=site_id,
            name=cls._name(model, mujoco.mjtObj.mjOBJ_SITE, site_id),
            body_id=int(model.site_bodyid[site_id]),
            position=np.asarray(model.site_pos[site_id], dtype=float).copy(),
            rotation=rotation_from_quaternion(model.site_quat[site_id]),
        )

    def _body_path(self, body_id):
        """월드 body 다음 노드부터 지정 body까지의 ID 경로를 부모 순서로 반환한다."""
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


    # 입력 : qpos (관절 위치), site_id (사이트 ID), joint_ids (관절 ID 목록)
    # 여기서 site_id는 계산하려는 site의 ID이며, joint_ids는 Jacobian을 계산할 때 고려할 관절들의 ID 목록이다.
    # site id가 뭐냐면 site는 MuJoCo에서 정의된 특정 지점이나 좌표계를 나타내며, site_id는 그 site를 식별하는 고유한 정수 값이다.
    # joint_ids는 Jacobian을 계산할 때 어떤 관절들을 포함할지 지정하는 리스트나 배열이다. Jacobian은 선택된 관절들에 대한 site의 위치 변화를 나타내므로, joint_ids를 통해 어떤 관절들이 영향을 미치는지 선택할 수 있다.
    # 출력 : site의 world pose와 선택된 joint 열의 6×N Jacobian
    def forward_site(self, qpos, site_id, joint_ids):
        """site world pose와 선택된 joint 열의 6×N Jacobian을 반환한다."""
        # site Jacobian은 point Jacobian과 hinge joint axis를 합쳐서 만든다.
        # site Jacobian은 6×N 행렬로, 상위 3행은 site의 위치 변화에 대한 Jacobian이고, 하위 3행은 site의 회전 변화에 대한 Jacobian이다. 이를 위해 먼저 point Jacobian을 계산하고, hinge joint의 경우에는 회전 축을 고려하여 하위 3행을 채운다.
        qpos = np.asarray(qpos, dtype=float)
        # 만약 qpos가 올바른 shape이 아니면 ValueError를 발생시킨다.
        if qpos.shape != (self.nq,):
            raise ValueError(f"expected qpos shape ({self.nq},), got {qpos.shape}")
        # site_id가 올바른 범위에 있는지 확인한다.
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
