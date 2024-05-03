# drone-flies-ISR

## Sources

| | System | ROS 2 | Python |
| ------- | ------- | ------- | ------ |
| GO1 | Ubuntu 22.04 | Humble | / |
| Crazyflies | Ubuntu 22.04 | Humble | 3.10 |

- GO1 :
  - https://www.youtube.com/watch?v=YSedTUxI0wc&ab_channel=DroneBlocks
    - https://gist.github.com/dbaldwin/feb0d279c67e0bcb191d2b366f867a84
    - https://community.droneblocks.io/t/go1-development-with-ros2-c-and-python/679/4
      > - ~~https://github.com/unitreerobotics/unitree_ros2_to_real~~ (deprecated)
      > - ~~https://github.com/unitreerobotics/unitree_legged_sdk/tree/f3b318a691e744e28caf6787eec90288f4016e87~~ [^1] (deprecated) 
      > - https://github.com/katie-hughes/unitree_ros2 (replace the previous 2 links)
      > - ~~https://github.com/lcm-proj/lcm/releases~~ (Not needed anymore with 3.8.0+)

[^1]: https://github.com/unitreerobotics/unitree_legged_sdk/tree/go1 is the most recent version

- Crazyflies :
  - https://github.com/IMRCLab/crazyswarm2

## Dependencies

- [Python](https://www.python.org/) (3.10+)
- [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
- Java 8
  > - `sudo apt install openjdk-8-jdk`
  > - `sudo update-alternatives --config javac`
  > (Then choose java-8-openjdk option)

### GO1
- **https://github.com/katie-hughes/unitree_ros2**

- [**unitree_ros2_to_real**](https://github.com/unitreerobotics/unitree_ros2_to_real)
- [**unitree_legged_sdk**](https://github.com/unitreerobotics/unitree_legged_sdk/tree/f3b318a691e744e28caf6787eec90288f4016e87)
  - [LCM](https://lcm-proj.github.io/lcm/) (1.4.0+)
  - [Boost](https://www.boost.org) (1.5.4+)
  - [CMake](https://www.cmake.org) (2.8.3+)
  - [g++](https://gcc.gnu.org) (8.3.0+)
  - (For 3.5.1 maybe the same for 3.8.6) Rename the unzipped folder from unitree_legged_sdk-3.5.1 to unitree_legged_sdk-master.
    Move the unitree_legged_sdk-master folder to ws/src/unitree_ros2_to_real
