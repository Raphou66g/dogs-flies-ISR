"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from crazyflie_py import Crazyswarm
from .class_position import Position
from .utils import formation, MinimalSubscriber


TAKEOFF_DURATION = 2.5
HOVER_DURATION = 2.5


def main():
    swarm = Crazyswarm()
    time_helper = swarm.timeHelper
    allcfs = swarm.allcfs
    allcfs.setParam("colAv.enable", 1)
    
    allcfs.takeoff(targetHeight=0.5, duration=TAKEOFF_DURATION)
    time_helper.sleep(TAKEOFF_DURATION)

    try:
        while True:
            pos = Position(allcfs.modo_x, allcfs.modo_y, allcfs.modo_z)
            form_coords = formation(len(allcfs.crazyflies), pos)

            for cf, coords in zip(allcfs.crazyflies, form_coords):
                cf.goTo(coords.array(), 0, 2.5)
            
            time_helper.sleep(HOVER_DURATION)

    except Exception as err:
        # Stop drones when something bad happens (raised exception).
        print(Exception, err)

    allcfs.land(targetHeight=0.04, duration=2.5)
    time_helper.sleep(TAKEOFF_DURATION)

if __name__ == '__main__':
    main()
