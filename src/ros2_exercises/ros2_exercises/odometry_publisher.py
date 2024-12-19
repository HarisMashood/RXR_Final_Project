#!/usr/bin/env python3
import math
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_euler

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class OdometryPublisher(Node):

    def __init__(self):
        super().__init__("odometry_publisher")
        # Use these wheel parameters for odometry calculation on Andino
        self.wheel_separation = 0.137
        self.wheel_radius = 0.033

        # Initializing the robot position and time for robot odometry calculation
        self.x = 0.0  # Robot's position on x-axis
        self.y = 0.0  # Robot's position on y-axis
        self.theta = 0.0  # Robot's orientation angle
        self.last_time = self.get_clock().now()

        # Subscribe to /joint_states topic to listen to data from left and right wheels.
        #  The message type is JointState and quality of service can be set to 10.
        #  Use self.joint_states_callback as the subscription callback.
        self.joint_subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10
        )

        # Create odometry publisher. Message type is Odometry, topic can be set to
        #  /robot_odometry (not to clash with the existing Andino odometry topic) and qos to 10.
        self.odom_publisher = self.create_publisher(Odometry, "/robot_odometry", 10)

        self.path_visualizer = PathVisualizer()

    def joint_states_callback(self, joint_states):
        # Read left and right wheel velocities from the JointState message
        joint_names = joint_states.name
        left_wheel_index = joint_names.index("left_wheel_joint")
        right_wheel_index = joint_names.index("right_wheel_joint")
        left_wheel_vel = joint_states.velocity[left_wheel_index]
        right_wheel_vel = joint_states.velocity[right_wheel_index]

        # Get the elapsed time since last odometry calculation, delta time (dt), in seconds.
        #  Save current time to self.last_time
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # The wheel velocities we read from the joint_states message are angular
        #  joint velocities (rad/s). Convert them to linear velocities for each wheel by multiplying
        #  them with a wheel radius. Then calculate the robot's linear and angular velocities
        #  with the following formulas:
        #  linear velocity = (vel right + vel left) / 2.0
        #  angular velocity = (vel right - vel left) / wheel separation
        v_left = left_wheel_vel * self.wheel_radius
        v_right = right_wheel_vel * self.wheel_radius
        linear_velocity = (v_right + v_left) / 2.0
        angular_velocity = (v_right - v_left) / self.wheel_separation

        # Now that we know how much time has elapsed since the last calculation,
        #  what was robot's previous orientation angle (theta) and with what speed the
        #  robot has moved, we can calculate the new position for the robot. Find out how to
        #  calculate this for x-axis, y-axis and robot's orientation theta, and
        #  update the values in self.x, self.y and self.theta.
        delta_x = linear_velocity * math.cos(self.theta) * dt
        delta_y = linear_velocity * math.sin(self.theta) * dt
        delta_theta = angular_velocity * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Create new Odometry message and populate stamp and frame IDs. The parent frame
        #  ID is "odom" and child frame ID is "base_link".
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Add the updated robot's position and orientation to the Odometry message
        # Be careful, the Odometry message accepts the orientation in Quaternion notation!
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_quat = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(
            x=odom_quat[0], y=odom_quat[1], z=odom_quat[2], w=odom_quat[3]
        )

        # Add the updated linear and angular velocities in the Odometry message
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = angular_velocity

        # Publish the odometry message
        self.odom_publisher.publish(odom_msg)

        self.path_visualizer.visualize(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y)


class PathVisualizer:
    def __init__(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()

        # Lock the aspect ratio and set equal scaling for x and y
        self.ax.set_aspect('equal', adjustable='box')

        self.path_x, self.path_y = [], []

    def visualize(self, x, y):
        # Store position history
        self.path_x.append(x)
        self.path_y.append(y)

        # Plot path
        self.ax.clear()
        self.ax.plot(self.path_x, self.path_y, 'b-', label='Path')
        self.ax.plot(x, y, 'ro', label='Current Position')  # Mark current position
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.set_title('Traversed Path')
        self.ax.legend()
        plt.draw()
        plt.pause(0.001)  # Short pause to update plot


def main(args=None):
    rclpy.init(args=args)

    odometry_publisher = OdometryPublisher()

    try:
        rclpy.spin(odometry_publisher)
    except KeyboardInterrupt:
        pass

    odometry_publisher.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
