class Position:

    def __init__(self, x: float, y: float, z: float, *, bound=True, min_x=-1., max_x=1., min_y=-1., max_y=1., min_z=0.4, max_z=1.):
        """
        Class to represent a position. All args are converted to float.
        min & max values are used to restrain drone's displacement to a certain zone.

        :param x: x position in the world
        :param y: y position in the world
        :param z: z position in the world

        :param bound: (Optional) define if min/max are used. Default True
        :param min_x: (Optional) set the minimum for the x value
        :param max_x: (Optional) set the maximum for the x value
        :param min_y: (Optional) set the minimum for the y value
        :param max_y: (Optional) set the maximum for the y value
        :param min_z: (Optional) set the minimum for the z value
        :param max_z: (Optional) set the maximum for the z value
        """
        self.x = min(max(min_x, x), max_x) if bound else x
        self.y = min(max(min_y, y), max_y) if bound else y
        self.z = min(max(min_z, z), max_z) if bound else z

