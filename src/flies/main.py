import json
import os

from environ_controller import EnvironController


"""
This is the main module where the game starts.
"""


def load_drones_settings():
    """
    Load drone settings from the json file.

    :return: List of dicts representing drone settings.
    """
    with open(f"{os.path.dirname(__file__)}/drones.json", "r") as f:
        drones = json.load(f)
    return drones


if __name__ == "__main__":
    
    while True:
        print("")
        print("1. Configure (install requirements)")
        print("2. Skip")
        st = input("Option n° : ")
        states = {"1": 1, "2": 2}
        state = states.get(st, 0)
        if state != 0:
            if state == 1:
                from configure import install_all
                install_all()
            break

    drones = load_drones_settings()

    while True:
        print("")
        print("Select an option :")
        print("1. ROS mode")
        print("2. Manual input mode")
        opt = input("Option n° : ")
        modes = {"1": 1, "2": 2}
        mode = modes.get(opt, 0)

        if mode != 0:
            break

    EnvironController(drones).main(mode=mode)
