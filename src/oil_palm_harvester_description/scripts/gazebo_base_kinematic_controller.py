#!/usr/bin/env python3
"""Kinematically move the Gazebo harvester and publish world->base_link TF.

This is deliberately a simple non-physical base controller for the first
sensor-simulation stage.  It accepts geometry_msgs/Twist on
/harvester/cmd_vel, moves the Gazebo model, and keeps RViz in the same pose.
"""

import math
import sys
import time

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class BaseKinematicController(Node):
    def __init__(self, model_name: str):
        super().__init__('gazebo_base_kinematic_controller')
        self.model_name = model_name
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_update = time.monotonic()
        self.broadcaster = TransformBroadcaster(self)
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.in_flight = False
        self.create_subscription(Twist, '/harvester/cmd_vel', self.velocity_callback, 10)
        self.create_timer(0.05, self.update)
        self.get_logger().info(
            'Kinematic base control ready: publish geometry_msgs/Twist to /harvester/cmd_vel')

    def velocity_callback(self, message: Twist):
        self.linear_x = message.linear.x
        self.angular_z = message.angular.z

    def update(self):
        now = time.monotonic()
        dt = now - self.last_update
        self.last_update = now
        self.x += self.linear_x * math.cos(self.yaw) * dt
        self.y += self.linear_x * math.sin(self.yaw) * dt
        self.yaw += self.angular_z * dt
        self.publish_transform()
        if self.in_flight or not self.client.service_is_ready():
            return
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self.model_name
        request.state.reference_frame = 'world'
        request.state.pose.position.x = self.x
        request.state.pose.position.y = self.y
        request.state.pose.position.z = 0.05
        request.state.pose.orientation.z = math.sin(self.yaw / 2.0)
        request.state.pose.orientation.w = math.cos(self.yaw / 2.0)
        request.state.twist.linear.x = self.linear_x * math.cos(self.yaw)
        request.state.twist.linear.y = self.linear_x * math.sin(self.yaw)
        request.state.twist.angular.z = self.angular_z
        self.in_flight = True
        future = self.client.call_async(request)
        future.add_done_callback(self.done_callback)

    def publish_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.05
        transform.transform.rotation.z = math.sin(self.yaw / 2.0)
        transform.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.broadcaster.sendTransform(transform)

    def done_callback(self, future):
        self.in_flight = False
        try:
            response = future.result()
            if not response.success:
                self.get_logger().warning(
                    f'Gazebo rejected base pose: {response.status_message}')
        except Exception as error:
            self.get_logger().warning(f'Gazebo base pose call failed: {error}')


def main():
    model_name = sys.argv[1] if len(sys.argv) > 1 else 'oil_palm_harvester'
    rclpy.init()
    node = BaseKinematicController(model_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
