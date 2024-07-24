"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from crazyflie_py import Crazyswarm

import numpy as np

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0


class Position:
    def __init__(
            self,
            x: float = 0,
            y: float = 0,
            z: float = 0.5
    ):
        """
        Class to represent a position. All args are converted to float.

        :param x: x position in the world
        :param y: y position in the world
        :param z: z position in the world
        """
        self.x = x
        self.y = y
        self.z = z

    def array(self):
        return np.array([self.x, self.y, self.z])

    def __str__(self) -> str:
        attr: str = ""
        for k, v in self.__dict__.items():
            attr += f"{k}={v},"
        return f"{self.__class__.__name__}({attr[:-1]})"


def formation(n: int, pos: Position):
    """
    Generate a distributed drone formation around an origin depending on how many drone we have

    :param n: (integer) Number of drone.
    :param pos: (Position) Center of the formation.

    :return: list[Position] - List of where each drone should be placed around the origin.
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
            a = angle if angle <= 90 else 180 - angle
            y = np.cos(np.deg2rad(a)) * space_h
            x = np.sin(np.deg2rad(a)) * space_h

            if angle <= 90:
                coord.append(Position(origin_x + x, origin_y + y, origin_z))
                coord.append(Position(origin_x - x, origin_y + y, origin_z))
            else:
                coord.append(Position(origin_x + x, origin_y - y, origin_z))
                coord.append(Position(origin_x - x, origin_y - y, origin_z))

        if n % 2 == 0:
            coord.append(Position(origin_x, origin_y - space_h, origin_z))

    return coord


def main():
    pos = Position()

    swarm = Crazyswarm()
    time_helper = swarm.timeHelper
    allcfs = swarm.allcfs
    allcfs.setParam("colAv.enable", 1)

    allcfs.takeoff(targetHeight=0.5, duration=TAKEOFF_DURATION)
    time_helper.sleep(TAKEOFF_DURATION)

    form_coords = formation(len(allcfs.crazyflies), pos)

    for cf, coords in zip(allcfs.crazyflies, form_coords):
        cf.goTo(coords.array(), 0, 3.0)

    time_helper.sleep(HOVER_DURATION)

    pos = Position(1.0, 0.0, 0.5)

    form_coords = formation(len(allcfs.crazyflies), pos)

    for cf, coords in zip(allcfs.crazyflies, form_coords):
        cf.goTo(coords.array(), 0, 3.0)

    time_helper.sleep(HOVER_DURATION)

    pos = Position(-1.0, 2.0, 1.0)

    form_coords = formation(len(allcfs.crazyflies), pos)

    for cf, coords in zip(allcfs.crazyflies, form_coords):
        cf.goTo(coords.array(), 0, 3.0)

    time_helper.sleep(HOVER_DURATION)

    allcfs.land(targetHeight=0.04, duration=2.5)
    time_helper.sleep(TAKEOFF_DURATION)


if __name__ == '__main__':
    main()
