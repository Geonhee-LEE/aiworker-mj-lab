"""Selected-joint forward kinematics for offline checks and demonstrations."""

from pathlib import Path

import mujoco
import numpy as np

from .tree import KinematicTree


class JointSpaceKinematics:
    """Evaluate one site over a selected set of scalar joints.

    Runtime controllers share :class:`KinematicTree` directly. This adapter is
    for offline FK/Jacobian checks that operate on a compact joint vector.
    """

    def __init__(self, model, site_name, joint_names, *, tree=None):
        self.model = model
        self.tree = KinematicTree(model) if tree is None else tree
        joint_names = tuple(joint_names)
        try:
            site = self.tree.site_by_name[site_name]
            joints = tuple(self.tree.joint_by_name[name] for name in joint_names)
        except KeyError as error:
            raise ValueError(
                "joint-space kinematics references an unknown site or joint: "
                f"{error.args[0]!r}"
            ) from error
        scalar_types = {
            int(mujoco.mjtJoint.mjJNT_HINGE),
            int(mujoco.mjtJoint.mjJNT_SLIDE),
        }
        unsupported = [joint.name for joint in joints if joint.kind not in scalar_types]
        if unsupported:
            raise ValueError(
                "controlled joints must be scalar hinge/slide joints: "
                + ", ".join(unsupported)
            )
        if len({joint.id for joint in joints}) != len(joints):
            raise ValueError("controlled joint names must be unique")

        self.site_name = str(site_name)
        self.joint_names = joint_names
        self.site_id = site.id
        self.joint_ids = np.array([joint.id for joint in joints], dtype=int)
        self.qpos_adrs = np.array([joint.qpos_adr for joint in joints], dtype=int)
        self.joint_ranges = np.array([joint.range for joint in joints], dtype=float)
        self.joint_limited = np.array([joint.limited for joint in joints], dtype=bool)
        self.n = len(joints)

    @classmethod
    def from_mjcf(cls, path, site_name, joint_names, **kwargs):
        model = mujoco.MjModel.from_xml_path(str(Path(path)))
        return cls(model, site_name, joint_names, **kwargs)

    def forward(self, q, context_qpos=None):
        q = np.asarray(q, dtype=float)
        if q.shape != (self.n,):
            raise ValueError(f"expected {self.n} joint positions, got {q.shape}")
        if context_qpos is None:
            qpos = self.tree.qpos0.copy()
        else:
            qpos = np.asarray(context_qpos, dtype=float).copy()
            if qpos.shape != (self.tree.nq,):
                raise ValueError(
                    f"expected context_qpos shape ({self.tree.nq},), got {qpos.shape}"
                )
        q = q.copy()
        q[self.joint_limited] = np.clip(
            q[self.joint_limited],
            self.joint_ranges[self.joint_limited, 0],
            self.joint_ranges[self.joint_limited, 1],
        )
        qpos[self.qpos_adrs] = q
        return self.tree.forward_site(qpos, self.site_id, self.joint_ids)

    forward_kinematics = forward


__all__ = ["JointSpaceKinematics"]
