# subscribe to: /hand/joint_command (sensor_msgs/JointState, target angles in rad)
# publishes: /hand/sim_joint_states (sensor_msgs/JointState, what sim does)

import mujoco
import mujoco.viewer

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import JointState

from mujoco_bridge import apply_command, build_actuator_map, read_joint_positions

class MujocoHandNode(Node):
    def __init__(self):
        super().__init__('mujoco_hand_node')

        self.declare_parameter('model_path', '') # repaklce with model path
        self.declare_parameter('command_topic', '/hand/joint_command')
        self.declare_parameter('state_topic', '/hand/sim_joint_states')
        self.declare_parameter('viewer', True)
        self.declare_parameter('tick_rate', 60.0)

        path = self.get_parameter('model_path').value
        if not path:
            raise RuntimeError("set the model_path parameter to MJCF file")

        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.JfData(self.model)
        self.act_map = build_actuator_map(self.model)
        if not self.act_map:
            raise RuntimeError("Model has no joint actuators")

        tick = 1.0 / self.get_parameter('tick_rate').value
        # use ros2 tick rate
        self.steps_per_tick = max(1, int(round(tick / self.model.opt.timestep)))

        self.warned = set()
        self.sub = self.create_subscription(
            JointState, 
            self.get_parameter('command_topic').value,
            self.on_command, 10)
        self.pub = self.create_publisher(
            JointState,
            self.get_parameter('state_topic').value,
            10)

        self.viewer = None
        if self.get_parameter('viewer').value:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
    
    def on_command(self, msg):
        unmatched = apply_command(self.model, self.data, self.act_map, msg.name, msg.position)
        new = [n for n in unmatched if n not in self.warned]
        if new:
            self.warned.update(new)
            self.get_logger().warn(f'No actuator for joints (ignored): {new}')

    def on_tick(self):
        if self.viewer is not None and not self.viewer.is_running():
            self.get_logger().info('Viewer closed, shutting down')
            rclpy.shutdown()
            return

        for _ in range(self.steps_per_tick):
            mujoco.mj_step(self.model, self.data)

        names, pos = read_joint_positions(self.model, self.data, list(self.act_map))
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name, out.position = names, pos
        self.pub.publish(out)

        if self.viewer is not None:
            self.viewer.sync()
        super().destroy_node()

    def main(args=None):
        rclpy.init(args=args)
        node = MujocoHandNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    if __name__ == '__main__':
        main()

        
