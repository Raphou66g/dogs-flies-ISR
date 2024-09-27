# drone-flies-ISR


- [📰 Sources](#sources)
- [🛠️ Dependencies](#dependencies)
  - [General](#dependencies-general)
  - [🐕 Go1](#dependencies-go1)
    - [Jetson Nano & ZED mini](#jetson)
  - [🪰 Crazyflies](#dependencies-flies)
  - [🚁 PX4 Vision](#️dependencies-px4)
- [From scratch](#scratch)
- [⌨️ Initialization](#initialization)
  - [🐕 Go1](#initialization-go1)
  - [🪰 Crazyflies](#initialization-flies)
  - [🚁 PX4 Vision](#initialization-px4)

## 📰 Sources <a id="sources"></a>

- GO1 :
  - [x] **<https://github.com/snt-arg/unitree_ros>** (The most recent depot working extremely well. We'll base our work on this)
  - [ ] <https://www.youtube.com/watch?v=YSedTUxI0wc&ab_channel=DroneBlocks>
    - <https://gist.github.com/dbaldwin/feb0d279c67e0bcb191d2b366f867a84>
    - <https://community.droneblocks.io/t/go1-development-with-ros2-c-and-python/679/4>
      > - <https://github.com/katie-hughes/unitree_ros2> (replace the next 2 links)
      > - <https://github.com/unitreerobotics/unitree_ros2_to_real>
      > - <https://github.com/unitreerobotics/unitree_legged_sdk/tree/f3b318a691e744e28caf6787eec90288f4016e87> [^1]
      > - <https://github.com/lcm-proj/lcm/releases> (Not needed anymore only if using unitree_legged_sdk 3.8.0+)

[^1]: <https://github.com/unitreerobotics/unitree_legged_sdk/tree/go1> is the most recent version

- Crazyflies :
  - <https://github.com/IMRCLab/crazyswarm2>

## 🛠️ Dependencies <a id="dependencies"></a>

| System | ROS 2 | Python |
| ------- | ------- | ------ |
| Ubuntu 22.04 | Humble | 3.10 |

### General <a id="dependencies-general"></a>

- [Python](https://www.python.org/) (3.10+)
- [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
  - (follow the instruction on https://docs.ros.org/en/humble/Installation.html then use the followings if needed.)

    ```bash
    sudo apt install ros-humble-xacro
    sudo apt install ros-humble-joint-state-publisher
    sudo apt install ros-humble-joint-state-publisher-gui
    ```


### 🐕 GO1 <a id="dependencies-go1"></a>

No specific dependencies other than the ones above.

#### Jetson Orin Nano & ZED mini <a id="jetson"></a>

The Go1's cameras aren't suited for SLAM, so we opted for a ZED Mini stereo camera, which we connected to a Jetson Orin Nano. The whole system will be wrapped on the Go1's back.

- NVIDIA SDK Manager :
  - [Download Link](https://developer.nvidia.com/sdk-manager)
- ZED SDK :
  - [Download Link](https://download.stereolabs.com/zedsdk/4.1/l4t36.3/jetsons)

### 🪰 Crazyflies <a id="dependencies-flies"></a>

The prerequisites are :
  - At least 1 Crazyflie 2.1 or more (None if simulation is used)
  - 1 Crazyradio PA
  - Ros 2 Humble installed

This part requires the Crazyswarm2 API. You can either run the `crazyswarm.sh` script or follow this tutorial for more information : https://imrclab.github.io/crazyswarm2/installation.html

> [!WARNING]  
> Please refer to steps 1, 2 and 5 to 7.  
> If you follow the tutorial, you can skip steps 3 and 4, as the ROS 2 workspace is already this workspace.  
> To use simulation, option point 7 is no longer optional.  

> [!TIP]  
> For the simulation, make sure that the command  
> `export PYTHONPATH=<replace-with-path-to>/crazyflie-firmware/build:$PYTHONPATH`  
> has been executed at least once after each machine reboot.
> Refers to step 7 for more information.

### 🚁 PX4 Vision <a id="dependencies-px4"></a>

This section is in an early state.

All you need for now are a PX4 Vision drone Kit, a Wi-Fi connection and some Python libraries :

```bash
pip3 install mavsdk
```

## From scratch <a id="scratch"></a>

### GO1

Other people have been working with the Go1 and we don't know if they have modified its code, but we suppose it should work properly even after a factory reset.

### Flashing and configuring the Jetson Orin Nano

We recommend you to use the [NVIDIA SDK Manager](https://developer.nvidia.com/sdk-manager) and simply following the steps.

When it asks for the card type, in our case, we choose the __Orin Nano 8Go (Developer Kit)__


## ⌨️ Initialization <a id="initialization"></a>

```bash
git clone --recursive https://github.com/Raphou66g/dogs-flies-ISR
cd dogs-flies-ISR
sh crazyswarm.sh
unzip -o src.zip
make build
```

> [!NOTE]  
> If you have any problems during this part, especially with the `crazyswarm.sh` script, you can always copy/paste each command line directly into your terminal.


### 🐕 GO1 <a id="initialization-go1"></a>

- Power up the Unitree Go1.
- If you can't wire your PC to the Go1 via Ethernet, connect the former to the 5GHz network named `Unitree_Go*******`. The default password is 00000000.
- Then, input the following command to launch the driver :
```bash
ros2 launch unitree_ros unitree_driver_launch.py 
```

> [!NOTE]  
> Add `wifi:=true` if using Wi-Fi

- The Go1 should stand up. You can now send ROS2 nodes to it.
- For example, test the following node, which should make the Go1 move forward by 2 meters : 
```bash
ros2 run unitree_ros DriverNode.py
```

### 🪰 Crazyflies <a id="initialization-flies"></a>

Don't forget to plug the Crazyradio to a USB port.

You can modify the number of drones and their configuration by editing the file `src/crazyswarm2/crazyflie/config/crazyflies.yaml`.

```yaml
robots:
  cf2:  # Label of the drone
    enabled: true  # Enable or not
    uri: radio://0/125/2M/E7E7E7E702  # Radio address of the drone. The last 2 digits are the id of the drone
    initial_position: [0.0, 0.0, 0.0] # x,y,z. Must be floats
    type: cf21  # see robot_types
```

You can test the sim with the following command line :

```bash
ros2 launch crazyflie_examples launch.py script:=hello_world backend:=sim
```

The specific scripts are `form_goto`, which moves the drone formation to different locations, and `form_ros2`, which waits for the coordinates of a specific ros subject (the `odom` topic).

```bash
ros2 launch crazyflie_examples launch.py script:=form_ros2 backend:=sim
```

To send different coordinates to the `form_ros2` script, you can run the `send_coords.py` file located in `src/crazyswarm2/crazyflie_examples/crazyflie_examples/send_coords.py` along with the other scripts, or you can move the Go1 after connecting it using [those instructions](#initialization-go1).

Delays are to be expected between the movement of the Go1 (or the sending of coordinates) and the movement of the drone.

> [!IMPORTANT]  
> The origin and orientation of the Go1 depend on where it is placed at power-up.  
> The X axis is in front of him and the Y axis to his left.  
> Standing up, the position is not really 0,0

The next step, which has not yet been tested, is to switch from the sim backend to cflib and take control of the real Crazyflies.

### 🚁 PX4 Vision <a id="initialization-px4"></a>

This section is in an early state.\
This part is still under development.



