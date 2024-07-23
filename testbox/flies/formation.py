from position import Position
import numpy as np
import matplotlib.pyplot as plt


def formation(n: int, pos: Position):
    """
    Generate a distributed drone formation around an origin depending on how many drone we have

    :param n: (integer) Number of drone.
    :param pos: (Position) Center of the formation.

    :return: list[Position] - List of where each drone should be place around the origin.
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


if __name__ == "__main__":
    lpos = formation(7, Position())
    print([str(p) for p in lpos])
    arr = [x.array() for x in lpos]
    L = []
    for val in arr:
        L.append([val[0], val[1]])
    x, y = zip(*L)
    plt.scatter(*zip(*L))
    plt.show()
