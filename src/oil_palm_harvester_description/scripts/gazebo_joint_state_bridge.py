#!/usr/bin/env python3
"""Apply the RViz joint-state GUI pose to the Gazebo harvester model."""

import sys

import rclpy
from gazebo_msgs.srv import SetModelConfiguration
from rclpy.node import Node
from sensor_msgs.msg import JointState


class GazeboJointStateBridge(Node):
    def __init__(self, topic: str, model_name: str):
        super().__init__('gazebo_joint_state_bridge')
        self.model_name = model_name
        self.client = self.create_client(
            SetModelConfiguration, '/gazebo/set_model_configuration')
        self.in_flight = False
        self.joint_names = []
        self.joint_positions = []
        self.pending = False
        self.subscription = self.create_subscription(
            JointState, topic, self.joint_state_callback, 10)
        self.create_timer(0.05, self.send_latest_pose)
        self.get_logger().info(
            f'Waiting to mirror {topic} into Gazebo model {model_name!r}')

    def joint_state_callback(self, message: JointState):
        if not message.name or len(message.name) != len(message.position):
            return
        self.joint_names = list(message.name)
        self.joint_positions = list(message.position)
        self.pending = True

    def send_latest_pose(self):
        if (self.in_flight or not self.pending or
                not self.client.service_is_ready()):
            return
        request = SetModelConfiguration.Request()
        request.model_name = self.model_name
        request.urdf_param_name = ''  # Unused in gazebo_msgs for ROS 2.
        request.joint_names = self.joint_names
        request.joint_positions = self.joint_positions
        self.pending = False
        self.in_flight = True
        future = self.client.call_async(request)
        future.add_done_callback(self.done_callback)

    def done_callback(self, future):
        self.in_flight = False
        try:
            response = future.result()
            if not response.success:
                self.get_logger().warning(
                    f'Gazebo rejected joint configuration: {response.status_message}')
        except Exception as error:
            self.get_logger().warning(f'Gazebo joint configuration call failed: {error}')


def main():
    # The combined Gazebo launch reserves /harvester/joint_states for
    # measured Gazebo feedback.  This legacy service bridge therefore follows
    # the GUI command topic if it is used outside the normal ModelPlugin path.
    topic = sys.argv[1] if len(sys.argv) > 1 else '/harvester/joint_commands'
    model_name = sys.argv[2] if len(sys.argv) > 2 else 'oil_palm_harvester'
    rclpy.init()
    node = GazeboJointStateBridge(topic, model_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
