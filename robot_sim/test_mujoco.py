"""Headless test: drive hand.xml through the bridge and check the joints actually follow."""

import os

import mujoco
import numpy as np

from mujoco_bridge import apply_command, build_actuator_map, read_joint_positions

HERE = os.path.dirname(os.path.abspath(__file__))
model = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'hand.xml'))
data = mujoco.MjData(model)
act_map = build_actuator_map(model)

# same order and values as teleop_node's joint_max
names = [f'{f}_{j}' for f in ('thumb', 'index', 'middle', 'ring', 'pinky')
         for j in ('mcp', 'pip', 'dip')]
joint_max = [1.0, 1.2, 1.0, 1.4, 1.7, 1.2, 1.4, 1.7, 1.2, 1.4, 1.7, 1.2, 1.4, 1.7, 1.2]


def settle(seconds=3.0):
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)


# 1) every command name maps to an actuator
assert set(names) == set(act_map), f'mismatch: {set(names) ^ set(act_map)}'

# 2) full curl: joints converge to joint_max
assert apply_command(model, data, act_map, names, joint_max) == []
settle()
_, pos = read_joint_positions(model, data, names)
err = np.max(np.abs(np.array(pos) - np.array(joint_max)))
print('full-curl max error (rad):', round(float(err), 4))
assert err < 0.08

# 3) open: joints return to zero
apply_command(model, data, act_map, names, [0.0] * 15)
settle()
_, pos = read_joint_positions(model, data, names)
err = np.max(np.abs(pos))
print('open max error (rad):', round(float(err), 4))
assert err < 0.08

# 4) out-of-range commands are clamped, unknown names are reported not crashed
apply_command(model, data, act_map, ['index_mcp'], [99.0])
assert data.ctrl[act_map['index_mcp']] == 2.0
assert apply_command(model, data, act_map, ['wrist_roll'], [0.1]) == ['wrist_roll']

# 5) no NaNs after all that
assert np.all(np.isfinite(data.qpos))
print('ALL MUJOCO BRIDGE TESTS PASSED')