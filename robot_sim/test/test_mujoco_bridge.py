"""Headless test: drive models/hand.xml through the bridge and check the joints actually follow.

Run with:  cd robot_sim && python3 -m pytest test      (or `colcon test --packages-select robot_sim`)
"""

import os

import mujoco
import numpy as np

from robot_sim.mujoco_bridge import apply_command, build_actuator_map, read_joint_positions

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, '..', 'models', 'hand.xml')

NAMES = [f'{f}_{j}' for f in ('thumb', 'index', 'middle', 'ring', 'pinky')
         for j in ('mcp', 'pip', 'dip')]
# same values as the teleop_node `joint_max` default (all inside the URDF limits)
JOINT_MAX = [1.0, 1.2, 1.0] + [1.4, 1.55, 1.2] * 4


def _load():
    model = mujoco.MjModel.from_xml_path(MODEL)
    return model, mujoco.MjData(model), build_actuator_map(model)


def _settle(model, data, seconds=3.0):
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)


def test_every_command_name_has_an_actuator():
    _, _, act_map = _load()
    assert set(NAMES) == set(act_map), f'mismatch: {set(NAMES) ^ set(act_map)}'


def test_full_curl_then_open():
    model, data, act_map = _load()
    assert apply_command(model, data, act_map, NAMES, JOINT_MAX) == []
    _settle(model, data)
    _, pos = read_joint_positions(model, data, NAMES)
    assert np.max(np.abs(np.array(pos) - np.array(JOINT_MAX))) < 0.08

    apply_command(model, data, act_map, NAMES, [0.0] * 15)
    _settle(model, data)
    _, pos = read_joint_positions(model, data, NAMES)
    assert np.max(np.abs(pos)) < 0.08


def test_clamp_unknown_and_finite():
    model, data, act_map = _load()
    apply_command(model, data, act_map, ['index_mcp'], [99.0])
    assert data.ctrl[act_map['index_mcp']] == model.actuator_ctrlrange[act_map['index_mcp']][1]
    assert apply_command(model, data, act_map, ['wrist_roll'], [0.1]) == ['wrist_roll']
    _settle(model, data, 1.0)
    assert np.all(np.isfinite(data.qpos))


def test_limits_match_urdf():
    """Every joint's MuJoCo range must equal the URDF limit, or RViz and MuJoCo would disagree."""
    import re
    urdf = os.path.join(HERE, '..', '..', 'robot_description', 'urdf', 'robot_hand.urdf')
    if not os.path.exists(urdf):
        return  # package tested on its own
    text = open(urdf).read()
    model, _, _ = _load()
    for name in NAMES:
        block = text[text.index(f'<joint name="{name}"'):]
        upper = float(re.search(r'upper="([^"]+)"', block).group(1))
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        assert abs(model.jnt_range[jid][1] - upper) < 1e-9, name
