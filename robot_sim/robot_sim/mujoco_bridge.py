# bridges mujoco without ros2 (for modularity)

import mujoco

def build_actuator_map(model):

    mapping = {}
    for a in range(model.nu):
        if int(model.actuator_trntype[a]) != int(mujoco.mjtTrn.mjTRN_JOINT):
            continue
        joint_id = int(model.actuator_trnid[a, 0])
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if name:
            mapping[name] = a
    return mapping

def apply_command(model, data, act_map, names, positions):
    unmatched = []
    for name, target in zip(names, positions):
        a = act_map.get(name)
        if a is None:
            unmatched.append(name)
            continue
        if model.actuator_ctrllimited[a]:
            lo, hi = model.actuator_ctrlrange[a]
            target = min(max(target, lo), hi)
        data.ctrl[a] = target
    return unmatched

def read_joint_positions(model, data, names):
    out_names, out_pos = [], []
    for name in names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            continue
        out_names.append(name)
        out_pos.append(float(data.qpos[model.jnt_qposadr[jid]]))
    return out_names, out_pos