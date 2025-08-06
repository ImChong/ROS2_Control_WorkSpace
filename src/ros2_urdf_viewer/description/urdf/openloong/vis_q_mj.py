import os
import sys
import time
import argparse
import pdb
import os.path as osp
import xml.etree.ElementTree as ET

sys.path.append(os.getcwd())

import torch

import numpy as np
import math
from copy import deepcopy
from collections import defaultdict
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation as sRot
import joblib

def add_visual_capsule(scene, point1, point2, radius, rgba):
    """Adds one capsule to an mjvScene."""
    if scene.ngeom >= scene.maxgeom:
        return
    scene.ngeom += 1  # increment ngeom
    # initialise a new capsule, add it to the scene using mjv_makeConnector
    mujoco.mjv_initGeom(scene.geoms[scene.ngeom-1],
                        mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3),
                        np.zeros(3), np.zeros(9), rgba.astype(np.float32))
    mujoco.mjv_makeConnector(scene.geoms[scene.ngeom-1],
                            mujoco.mjtGeom.mjGEOM_CAPSULE, radius,
                            point1[0], point1[1], point1[2],
                            point2[0], point2[1], point2[2])

def add_visual_arrow(scene, position, direction, length, radius, rgba):
    """Adds an arrow to an mjvScene."""
    if scene.ngeom >= scene.maxgeom:
        return
    # Increase shaft size
    shaft_end = position + direction * length * 0.75
    add_visual_capsule(scene, position, shaft_end, radius, rgba)

    # Create a larger cone substitute
    if scene.ngeom >= scene.maxgeom:
        return
    scene.ngeom += 1
    # Increase radius for a bigger head
    cone_radius = radius * 7  # Make the head wider
    cone_length = length * 0.25  # Increase head length
    # Approximate a cone using a cylinder
    mujoco.mjv_initGeom(scene.geoms[scene.ngeom - 1],
                        mujoco.mjtGeom.mjGEOM_CYLINDER,  # Use cylinder since cone is unavailable
                        np.zeros(3),
                        np.zeros(3), np.zeros(9), rgba.astype(np.float32))
    mujoco.mjv_makeConnector(scene.geoms[scene.ngeom - 1],
                            mujoco.mjtGeom.mjGEOM_CYLINDER,  # Fake a cone with a big cylinder
                            cone_radius,
                            shaft_end[0], shaft_end[1], shaft_end[2],
                            (shaft_end + direction * cone_length)[0],
                            (shaft_end + direction * cone_length)[1],
                            (shaft_end + direction * cone_length)[2])

def key_call_back( keycode):
    global curr_start, num_motions, motion_id, motion_acc, time_step, dt, paused, motion_data_keys
    if chr(keycode) == "R":
        print("Reset")
        time_step = 0
    elif chr(keycode) == " ":
        print("Paused")
        paused = not paused
    elif chr(keycode) == "T":
        print("next")
        motion_id += 1
        curr_motion_key = motion_data_keys[motion_id]
        print(curr_motion_key)
    else:
        print("not mapped", chr(keycode))
    
    
def main() -> None:
    global curr_start, num_motions, motion_id, motion_acc, time_step, dt, paused, motion_data_keys
    device = torch.device("cpu")
    humanoid_xml = "resources/robots/openloong/urdf/openloong_mujoco.xml"
    
    # motion_file = "scripts/read_csv/A2_taiji.pkl"
     
    # motion_file = "data/imitation_data/grad_fit_ik/A2/dance/boxing_24s.pkl"

    motion_file = "data/imitation_data/openloong/openloong_quan_with_aa_stand_contact_40s_0714.pkl"
    # motion_file = "scripts/read_csv/A2_taiji_new.pkl"
    print(motion_file)
    motion_data = joblib.load(motion_file)
    motion_data_keys = list(motion_data.keys())
    
    curr_start, num_motions, motion_id, motion_acc, time_step, dt, paused = 0, 1, 0, set(), 0, 1/motion_data[motion_data_keys[0]]['fps'], False
    
    mj_model = mujoco.MjModel.from_xml_path(humanoid_xml)
    mj_data = mujoco.MjData(mj_model)
    
    tree = ET.parse("resources/robots/openloong/urdf/openloong_mujoco.xml")
    root = tree.getroot()

    joint_names = []
    for joint in root.findall(".//joint"):
        joint_names.append(joint.get("name"))

    print(joint_names)
    RECORDING = False
    import matplotlib.pyplot as plt
    plt.plot(motion_data['openloong_dance']['dof_pos'][:, 22])
    plt.plot(motion_data['openloong_dance']['dof_pos'][:, 15])
    plt.show()
    mj_model.opt.timestep = dt
    with mujoco.viewer.launch_passive(mj_model, mj_data, key_callback=key_call_back) as viewer:
        for _ in range(50):
            add_visual_capsule(viewer.user_scn, np.zeros(3), np.array([0.001, 0, 0]), 0.05, np.array([1, 0, 0, 1]))
        # for _ in range(24):
        #     add_visual_capsule(viewer.user_scn, np.zeros(3), np.array([0.001, 0, 0]), 0.05, np.array([0, 1, 0, 1]))
        # Close the viewer automatically after 30 wall-seconds.
        count = 0
        while viewer.is_running():
            step_start = time.time()
            viewer.user_scn.ngeom = 0  
            curr_motion_key = motion_data_keys[motion_id]
            curr_motion = motion_data[curr_motion_key]
            curr_time = int(time_step/dt) % curr_motion['dof_pos'].shape[0]
            if curr_time % 60 == 0:
                # print("...")
                pass
            mj_data.qpos[:3] = curr_motion['root_trans_offset'][curr_time]
            # mj_data.qpos[2] -= 0.08 
            mj_data.qpos[3:7] = curr_motion['root_rot'][curr_time][[3, 0, 1, 2]]
            mj_data.qpos[7:] = curr_motion['dof_pos'][curr_time].squeeze()
            # mj_data.qpos[10:22] = curr_motion['dof_pos'][curr_time][:12].squeeze()
            # mj_data.qpos[22:] = curr_motion['dof_pos'][curr_time][15:].squeeze()
            # print(curr_motion['contact_flags'][curr_time])
            mujoco.mj_forward(mj_model, mj_data)
            if not paused:
                time_step += dt
            # Draw arrows if feet are in contact
            if curr_motion['contact_flags'][curr_time][0]:  # Left foot contact
                left_foot_pos = mj_data.xpos[mj_model.body("leg_l6_link").id]
                add_visual_arrow(viewer.user_scn, left_foot_pos, np.array([0, 0, 1]), 0.1, 0.01, np.array([1, 0, 0, 1]))
            if curr_motion['contact_flags'][curr_time][1]:  # Right foot contact
                right_foot_pos = mj_data.xpos[mj_model.body("leg_r6_link").id]
                add_visual_arrow(viewer.user_scn, right_foot_pos, np.array([0, 0, 1]), 0.1, 0.01, np.array([0, 1, 0, 1]))

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()
            time_until_next_step = mj_model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
            count += 1
            if (count % 5) == 0:
                print(f"Count: {count}")

            # if RECORDING and time_step > motion_len:
            #     curr_start += num_motions
            #     motion_lib.load_motions(skeleton_trees=[sk_tree] * num_motions, gender_betas=[torch.zeros(17)] * num_motions, limb_weights=[np.zeros(10)] * num_motions, random_sample=False, start_idx=curr_start)
            #     time_step = 0

if __name__ == "__main__":
    main()
