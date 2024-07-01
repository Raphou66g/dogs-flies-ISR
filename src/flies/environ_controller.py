import sys, select
import numpy as np
from time import sleep

from threading import Thread, Lock
from drone_controller import DroneController
from position import Position

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


def is_number(s: str):
    """_summary_

    Args:
        s (str): input String

    Returns:
        boolean: the String is an number or not
    """
    try:
        float(s)
        return True
    except ValueError:
        return False


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        self.pos = Position()
        self.twistL: Vector3 = None
        self.twistA: Vector3 = None
        self.orient: Quaternion = None
        self.subscription = self.create_subscription(
            Odometry, "odom", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

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


class EnvironController:
    """
    This class controls the whole volleyball game scenario, from ball projection to sending instructions to drones.

    :param flyers: (list) A list of dicts representing the drones (flyers).
    """

    def __init__(self, flyers: list[dict]):
        self.flyers = [DroneController(**flyer) for flyer in flyers]
        self.flyers_status = [False for _ in flyers]
        self.threads: list[Thread] = []
        self.alive: int = len(self.threads)
        self.lock = Lock()

    def start_drones(self):  # CORE
        """
        Creates a thread for each flyer and starts it.
        """

        for flyer in self.flyers:
            self.threads.append(Thread(target=flyer.main).start())

    def alive_flyers(self, duration: float = 5):
        """
        Auto verify if a drone is missing and autoremove it from the thread list.
        """
        while True:
            sleep(duration)

            with self.lock:
                self.threads = [thread for thread in self.threads if thread.is_alive()]
                self.alive = len(self.threads)

    def stop_drones(self):  # CORE
        """
        Stops all the flyers by telling the drones to land.
        """
        for flyer in self.flyers:
            flyer.land_now = True

    def get_next_flyer(self):  # CORE
        """
        This function checks all the flyers statuses and finds the next flyer.

        :return: (DroneController): Next flyer.
        """

        if all(self.flyers_status):
            self.flyers_status = [False for _ in self.flyers]

        for index in range(len(self.flyers_status)):
            if not self.flyers_status[index]:
                self.flyers_status[index] = True
                return self.flyers[index]

    def formation(self, n: int, pos: Position):
        """
        Generate a distributed drone formation around an origin depending on how many drone we have

        :param n: (integer) Number of drone.
        :param pos: (Position) Center of the formation.

        :return: list[Position] - List of where each drone should be place around the origin.
        """
        space_h = 0.5
        space_v = 1

        origin_x, origin_y, origin_z = pos.x, pos.y, pos.z + space_v
        coord = []

        if n == 1:
            coord = [Position(origin_x, origin_y, origin_z)]
        elif n == 2:
            coord = [
                Position(origin_x - space_h, origin_y, origin_z),
                Position(origin_x + space_h, origin_y, origin_z),
            ]
        else:
            ang = 360 / n
            coord = [Position(origin_x, origin_y + space_h, origin_z)]

            for i in range(1, (n // 2) + 1):
                angle = i * ang
                A = angle if angle <= 90 else 180 - angle
                y = np.cos(np.deg2rad(A)) * space_h
                x = np.sin(np.deg2rad(A)) * space_h

                if angle <= 90:
                    coord.append(Position(origin_x + x, origin_y + y, origin_z))
                    coord.append(Position(origin_x - x, origin_y + y, origin_z))
                else:
                    coord.append(Position(origin_x + x, origin_y - y, origin_z))
                    coord.append(Position(origin_x - x, origin_y - y, origin_z))

            if n % 2 == 0:
                coord.append(Position(origin_x, origin_y - space_h, origin_z))

        return coord

    def main(self, mode: int = 1, args=None):
        """Core of the environment controller

        :param mode: (int, optional) Chose between ROS mode (1) and manual mode (2). Defaults to 1.
        :param args: (_type_, optional) _description_. Defaults to None.
        """

        if mode == 0:
            return

        # ROS2 init
        rclpy.init(args=args)
        minimal_subscriber = MinimalSubscriber()

        form_coords = self.formation(len(self.flyers), minimal_subscriber.pos)
        for flyer, coord in zip(self.flyers, form_coords):
            flyer.position_to_visit = coord

        self.start_drones()

        try:
            while True:
                if mode == 1:
                    rclpy.spin_once(minimal_subscriber, timeout_sec=0.1)

                else:
                    print(
                        "enter X Y Z coords split with space. Floats allowed with . separator"
                    )
                    print(f"Actual coordinates : {minimal_subscriber.pos}")
                    i, _, _ = select.select([sys.stdin], [], [], 10)

                    if i:
                        try:
                            coords = sys.stdin.readline().strip().split(" ")
                            if len(coords) != 3:
                                raise ValueError(
                                    f"Wrong number of parameters. Need exactly 3 and got {len(coords)}"
                                )
                            for c in coords:
                                if not is_number(c):
                                    raise ValueError(f"{c} is not a valid number")
                            minimal_subscriber.pos.x = float(coords[0])
                            minimal_subscriber.pos.y = float(coords[1])
                            minimal_subscriber.pos.z = float(coords[2])
                        except Exception as e:
                            print("Warning : " + e)

                form_coords = self.formation(
                    len(self.alive_flyers()), minimal_subscriber.pos
                )
                # print([str(form_coords[i]) for i in range(len(form_coords))])

                # Send drone to the position
                for flyer, coord in zip(self.flyers, form_coords):
                    # print(coord)
                    flyer.position_to_visit = coord

        except Exception as err:
            # Stop drones when something bad happens (raised exception).
            self.stop_drones()
            print(Exception, err)

        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    import json

    def load_drones_settings():
        """
        Load drone settings from the json file.

        :return: List of dicts representing drone settings.
        """
        with open("drones.json", "r") as f:
            drones = json.load(f)
        return drones

    drones = load_drones_settings()
    EnvironController(drones).main(mode=1)
