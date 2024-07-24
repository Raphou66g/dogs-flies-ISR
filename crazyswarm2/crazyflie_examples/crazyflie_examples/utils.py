from .class_position import Position
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
)

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        self.pos = Position()
        self.twistL: Vector3 = None
        self.twistA: Vector3 = None
        self.orient: Quaternion = None
        self.subscription = self.create_subscription(
            Odometry, "modo", self.listener_callback, 10
        )

    def listener_callback(self, msg: Odometry):
        """Action at the reception of a ROS2 message

        :param msg: (Odometry) message received from the GO1
        """
        poseWC: PoseWithCovariance = msg.pose
        pose: Pose = poseWC.pose
        self.pos.x, self.pos.y, self.pos.z = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
        )
        self.orient = pose.orientation
        # twistWC: TwistWithCovariance = msg.twist
        # twist: Twist = twistWC.twist
        # self.twistL = twist.linear
        # self.twistA = twist.angular
        print(self.pos, self.orient)  # , self.twistL, self.twistA)



def formation(n: int, pos: Position):
    """
    Generate a distributed drone formation around an origin depending on how many drone we have

    :param n: (integer) Number of drone.
    :param pos: (Position) Center of the formation.

    :return: list[Position] - List of where each drone should be placed around the origin.
    """
    space_h = 0.5
    space_v = 0.5

    origin_x, origin_y, origin_z = pos.x, pos.y, pos.z + space_v
    coord = []

    if n == 1:
        coord = [Position(origin_x, origin_y, origin_z)]
    elif n == 2:
        coord = [
            Position(origin_x, origin_y - space_h, origin_z),
            Position(origin_x, origin_y + space_h, origin_z),
        ]
    else:
        ang = 360 / n
        coord = [Position(origin_x + space_h, origin_y, origin_z)]

        for i in range(1, (n // 2) + 1):
            angle = i * ang
            a = angle if angle <= 90 else 180 - angle
            x = np.cos(np.deg2rad(a)) * space_h
            y = np.sin(np.deg2rad(a)) * space_h

            if angle <= 90:
                coord.append(Position(origin_x + x, origin_y + y, origin_z))
                coord.append(Position(origin_x + x, origin_y - y, origin_z))
            else:
                coord.append(Position(origin_x - x, origin_y + y, origin_z))
                coord.append(Position(origin_x - x, origin_y - y, origin_z))

        if n % 2 == 0:
            coord.append(Position(origin_x - space_h, origin_y, origin_z))

    return coord
