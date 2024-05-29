#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscription_ = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.timer_ = self.create_timer(0.1, self.publish_cmd_vel)  # 0.1 second timer
        self.get_logger().info("Trajectory Follower node has started.")

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.orient_quat = (0.0, 0.0, 0.0, 0.0)
        self.roll_x = 0.0
        self.pitch_y = 0.0
        self.yaw_z = 0.0

        self.trajectory = [
            #(3.5, 0.0),
            #(3.5,9.2),
            #(0.0,9.2),
            (2.0, 0.0)
        ]

        self.current_waypoint = 0

    def odom_callback(self, msg):
        # Extracting the robot pose from the Odometry message
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        orient = msg.pose.pose.orientation
        self.roll_x, self.pitch_y, self.yaw_z = euler_from_quaternion(orient.x, orient.y, orient.z, orient.w)

    def publish_cmd_vel(self):
        current_waypoint = self.trajectory[self.current_waypoint]
        print("Moving to :", current_waypoint)
        print("Current position : x = ", self.pose_x, " y = ", self.pose_y)

        # Calculate the difference between the current position and the waypoint
        dx = current_waypoint[0] - self.pose_x
        dy = current_waypoint[1] - self.pose_y

        distance_to_waypoint = math.sqrt(dx ** 2 + dy ** 2)

        # Calculate the angle between the current heading and the vector to the waypoint
        angle_to_waypoint = math.atan2(dy, dx) - self.yaw_z

        linear_speed = 0.2
        msg = Twist()

        if distance_to_waypoint > 0.1:
            msg.linear.x = linear_speed
            msg.angular.z = math.atan2(math.sin(angle_to_waypoint), math.cos(angle_to_waypoint))  # Set angular velocity
            print("Distance to point : ", distance_to_waypoint)

        else:
            print("Arrivée au point")
            msg.linear.x = 0.0
            msg.angular.z = 0.0

            # Move to the next waypoint
            if self.current_waypoint < len(self.trajectory) - 1:
                self.current_waypoint += 1
                print("Current Waypoint", self.current_waypoint)

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    trajectory_follower = TrajectoryFollower()

    rclpy.spin(trajectory_follower)

    trajectory_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
