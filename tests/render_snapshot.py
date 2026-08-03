"""모델 자세를 눈으로 확인할 offscreen snapshot을 렌더링하는 개발 도구.

자동 Phase 테스트에는 포함되지 않는다.

사용법: ``python3 tests/render_snapshot.py models/hand_only.xml /tmp/out.png [--grasp 0.5]``
"""

import sys
import pathlib
import mujoco
import numpy as np
from PIL import Image

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent


def main():
    """CLI로 지정한 MJCF의 손 자세를 offscreen 렌더링해 PNG로 저장한다."""
    model_path = sys.argv[1]
    out_path = sys.argv[2]
    grasp = 0.0
    kinematic = False
    for a in sys.argv[3:]:
        if a.startswith("--grasp="):
            grasp = float(a.split("=")[1])
        if a == "--kinematic":
            kinematic = True

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    # 엄지의 MCP pitch·IP와 각 손가락의 PIP·DIP·tip 굽힘 관절만 포함한다.
    # 사전 자세인 thumb CMC·MCP yaw와 벌림 관절인 finger MCP는 0으로 두고 제외한다.
    CURL_JOINTS = {"finger_r_joint3", "finger_r_joint4"}
    for base in (5, 9, 13, 17):
        CURL_JOINTS.update({f"finger_r_joint{base+1}", f"finger_r_joint{base+2}", f"finger_r_joint{base+3}"})

    if kinematic:
        # 순수 FK 자세 확인을 위해 굽힘 관절 qpos를 관절 범위의 ``grasp`` 비율로 직접
        # 설정하고 스텝 진행 없이 ``mj_forward``를 한 번만 호출한다. 일회성 제작·시각화
        # snapshot이므로 런타임 시뮬레이션의 기구학 덮어쓰기 금지 규칙과는 구분된다.
        for jid in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if name in CURL_JOINTS:
                lo, hi = model.jnt_range[jid]
                qadr = model.jnt_qposadr[jid]
                data.qpos[qadr] = lo + grasp * (hi - lo)
        mujoco.mj_forward(model, data)
    elif grasp > 0:
        for aid in range(model.nu):
            jid = model.actuator_trnid[aid, 0]
            lo, hi = model.jnt_range[jid]
            data.ctrl[aid] = lo + grasp * (hi - lo)
        for _ in range(2000):
            mujoco.mj_step(model, data)

    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, height=720, width=960)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, cam)
    cam.lookat = np.array([0.05, 0.0, 0.15])
    cam.distance = 0.5
    cam.azimuth = 140
    cam.elevation = -20

    show_contacts = "--contacts" in sys.argv
    scene_opt = mujoco.MjvOption()
    if show_contacts:
        scene_opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
        scene_opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True

    renderer.update_scene(data, camera=cam, scene_option=scene_opt)
    pixels = renderer.render()
    Image.fromarray(pixels).save(out_path)
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
