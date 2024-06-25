import json

from environ_controller import GameController


"""
This is the main module where the game starts.
"""


def load_drones_settings():
    """
    Load drone settings from the json file.

    :return: List of dicts representing drone settings.
    """
    with open('drones.json', 'r') as f:
        drones = json.load(f)
    return drones
        

if __name__ == '__main__':
    drones = load_drones_settings()

    print("Select an option :")
    print("1. ROS mode")
    print("2. Manual input mode")
    opt = input("Option n° : ")

    modes = {"1": 1, "2": 2}
    mode = modes.get(opt, 0)
    GameController(drones).main(mode=mode)