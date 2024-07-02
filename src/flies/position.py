class Position:

    def __init__(
        self,
        x: float = 0,
        y: float = 0,
        z: float = 0.2
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

    def __str__(self) -> str:
        attr:str = ""
        for k,v in self.__dict__.items():
            attr += f"{k}={v},"
        return f"{self.__class__.__name__}({attr[:-1]})"