# drone-flies-ISR

- [📰 Sources](#sources)
- [🛠️ Dependencies](#dependencies)
  - [General](#dependencies-general)
  - [🐕 Go1](#dependencies-go1)
    - [Jetson Nano & ZED mini](#jetson)
  - [🪰 Crazyflies](#dependencies-flies)
- [From scratch](#scratch)
- [⌨️ Initialization](#initialization)


## 📰 Sources <a id="sources"></a>

- GO1 :
  - [x] **<https://github.com/snt-arg/unitree_ros>** (The most recent depot working extremly well. We'll base our work on this)
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

#### Jetson Nano & ZED mini <a id="jetson"></a>

The Go1's cameras aren't suited for SLAM, so we opted for a ZED Mini stereo camera, which we connected to a Jetson Nano. The whole system will be wrapped on the Go1's back.

- The JN30D carrier board's firmware :
  - [Mega](https://mega.nz/file/2YknhI6A#5s0Zr9UmwSfbIX-MFVpjSrUjrLEhWtQTiXN13qAWELM) | [Original Download Link](https://f000.backblazeb2.com/file/auvidea-download/images/Jetpack_4_6/BSP/Jetpack4.6_Nano_BSP.tar.gz)
- ZED SDK :
  - [Mega](https://mega.nz/file/HU1wSbxJ#px-IquGm5MQEqtKCeBQiUl40IVINUzJb41PFfCNexSk) | [Original Download Link](https://download.stereolabs.com/zedsdk/3.8/l4t32.6/jetsons)

### 🪰 Crazyflies <a id="dependencies-flies"></a>

TBD

## From scratch <a id="scratch"></a>

### GO1

Other people have been working with the Go1 and we don't know if they have modified its code, but we suppose it should work properly even after a factory reset.

### Flashing and configuring the Jetson Nano

The following steps have been done on Ubuntu 18.04. It should work with more recent versions but to avoid any problems we recommend using the same Ubuntu version.

The entire process can be found [here](./docs/Auvidea_Software.pdf). 

⚠️ Warning ⚠️ : There are mistakes in this file which we will address below.

- First, you need to flash the Jetson Nano.

  Download the Jetson's firmware. It can be found above ([here](#jetson)).

  Then, setup your Jetson to be in recovery mode, connect it to your computer and then run the downloaded file.

  ```bash
  sudo bash ./flashcmd.txt
  ```

- If your Jetson has an external storage like ours (in our case, a microSD), we recommend to use it and make the Jetson boot on it by following the "SECTION 5" on Auvidea's PDF.
  - Follow SECTION 5.1.2 on [Auvidea's guide](./docs/Auvidea_Software.pdf)

    In our case, it was `/dev/mmcblk1p1` so we removed the `p1` at the end and we got : 
    
    <YOUR_STORAGE_DEVICE> = `/dev/mmcblk1`

    <YOUR_STORAGE_PREFIX> = `p1`

  - Set up RootFS on SSD

    1. Format the storage device

        ```bash
        sudo parted <YOUR_STORAGE_DEVICE> mklabel gpt
        ```

    2. Create the RootFS partition

        ```bash
        sudo parted <YOUR_STORAGE_DEVICE> mkpart APP 0GB
        ```

        When parted asks for the end of the partition, input `100%` if you want to use the whole storage.

    3. Create filesystem

        ```bash
        sudo mkfs.ext4 <YOUR_STORAGE_DEVICE><YOUR_STORAGE_PREFIX>
        ```
        with no space between your variables.

    4. Copy the existing RootFS to the storage device.

        ```bash
        sudo mount <YOUR_STORAGE_DEVICE><YOUR_STORAGE_PREFIX> /mnt
        sudo rsync -axHAWX --numeric-ids --info=progress2 --exclude={"/dev/","/proc/","/sys/","/tmp/","/run/","/mnt/","/media/*","/lost+found"} / /mnt/
        ```
  - Switch boot device to the external device

    5. Modify the extlinux.conf by changing the root path

        ```bash
        sudo nano /boot/extlinux/extlinux.conf
        ```

        Then, locate the `LABEL primary` line and **ONLY** change the root path with `<YOUR_STORAGE_DEVICE><YOUR_STORAGE_PREFIX>` with no spaces :

        ```
        LABEL primary 
          MENU LABEL primary kernel
          LINUX /boot/Image
          INITRD /boot/initrd 
          APPEND ${cbootargs} quiet root=<YOUR_STORAGE_DEVICE><YOUR_STORAGE_PREFIX> rw rootwait rootfstype=ext4 console=ttyTCU0,115200n8 console=tty0 fbcon=map:0 net.ifnames=0
        ```
  - Reboot the Jetson.

    If every step has been done correctly, while using `df` in a terminal, you should be able to see <YOUR_STORAGE> as your filesystem.

  Auvidea <font color='red'>**recommends to not use**</font> `apt upgrade` on the Jetson as it could break some of its functionalities. 
  
- You can then install the JetPack SDK :

  ```bash
  sudo apt update
  sudo apt install nvidia-jetpack
  ```


## ⌨️ Initialization <a id="initialization"></a>

- Place yourself in the `/drone-flies-ISR` folder.
- `colcon build --symlink-install`
- Power up the Unitree Go1.
- If you can't wire your PC to the Go1 via Ethernet, connect the former to the 5GHz network named `Unitree_Go*******`. The default password is 00000000.
- Then, input the following command to launch the driver :
```bash
ros2 launch unitree_ros unitree_driver_launch.py # add wifi:=true if using WiFi
```
- The Go1 should stand up. You can now send ROS2 nodes to it.
- For example, test the following node, which should make the Go1 move forward by 2 meters : 
```bash
ros2 run unitree_ros DriverNode.py
```
