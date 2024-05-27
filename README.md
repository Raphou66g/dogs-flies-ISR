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

The GO1 cameras aren't the best ones for SLAM, we decided to use the ZED mini camera connected to a Jetson Nano wrap on his back.

- Jetson's firmware.
  - [Mega](https://mega.nz/file/2YknhI6A#5s0Zr9UmwSfbIX-MFVpjSrUjrLEhWtQTiXN13qAWELM) | [Original Download Link](https://f000.backblazeb2.com/file/auvidea-download/images/Jetpack_4_6/BSP/Jetpack4.6_Nano_BSP.tar.gz)
- ZED SDK
  - [Mega](https://mega.nz/file/HU1wSbxJ#px-IquGm5MQEqtKCeBQiUl40IVINUzJb41PFfCNexSk) | [Original Download Link](https://download.stereolabs.com/zedsdk/3.8/l4t32.6/jetsons)

### 🪰 Crazyflies <a id="dependencies-flies"></a>

TBD

## From Scratch <a id="scratch"></a>

### GO1

Other peoples has been working with it and we don't know if they'd done something on his code. It must works too even after a factory reset.

### Jetson Nano

For the Jetson Nano configuration, all these steps have been done on Ubuntu 18.04. It should work with more recent versions but to avoid any problems we recommand using the same Ubuntu version.

The entire processus can be found [here](./docs/Auvidea_Software.pdf)
⚠️ Warning ⚠️ : there's some error in this file but i'll provide you the correct step by step.

- You first need to flash it.

  Starts by downloading the Jetson's firmware. It can be found above ([here](#jetson)).

  Then connect your Jetson to your computer and run the downloaded file.

  ```bash
  sudo bash ./flashcmd.txt
  ```

  It could take some times.

- If your Jetson have an external storage like ours (SSD, SD card...), we recommand to use it and make it the boot system by following the "SECTION 5" on Auvidea's PDF. This section have some mistakes that i'll fix for you.

  - Follow 5.1.2 on [Auvidea's guide](./docs/Auvidea_Software.pdf)

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

        When it ask for the end of the partition, simply input `100%`.

    3. Create filesystem

        ```bash
        sudo mkfs.ext4 <YOUR_STORAGE_DEVICE><YOUR_STORAGE_PREFIX>
        ```
        without space between your variables.

    4. Copy the existing RootFS to the storage device.

        ```bash
        sudo mount <YOUR_STORAGE_DEVICE> /mnt
        sudo rsync -axHAWX --numeric-ids --info=progress2 --exclude={"/dev/","/proc/","/sys/","/tmp/","/run/","/mnt/","/media/*","/lost+found"} / /mnt/
        ```
  - Switch boot device to external device

    5. Modify the extlinux.conf by changing the root path

        We are using `nano` but feel free to use your prefered editor.

        ```bash
        sudo nano /boot/extlinux/extlinux.conf
        ```

        then locate the `LABEL primary` line and **ONLY** change the root path with `<YOUR_STORAGE_DEVICE><YOUR_STORAGE_PREFIX>` without space. Here's an exemple :

        ```
        LABEL primary 
          MENU LABEL primary kernel
          LINUX /boot/Image
          INITRD /boot/initrd 
          APPEND ${cbootargs} quiet root=<YOUR_STORAGE_DEVICE><YOUR_STORAGE_PREFIX> rw rootwait rootfstype=ext4 console=ttyTCU0,115200n8 console=tty0 fbcon=map:0 net.ifnames=0
        ```
  - Reboot and verify.

    If every steps are done successfuly, you would be able to see <YOUR_STORAGE> as filesystem under the `df /` commande on your terminal.

- Time to update your Jetson.

  ```bash
  sudo apt update
  ```

  Auvidea <font color='red'>**DO NOT RECOMMAND**</font> upgrading your system, so we don't.

## ⌨️ Initialization <a id="initialization"></a>

- Place yourself in the `/drone-flies-ISR` folder.
- `colcon build --symlink-install`
