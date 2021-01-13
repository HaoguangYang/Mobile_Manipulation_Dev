# Mobile_Manipulation_Dev
Development repo for the mobile manipulation platforms

### Robot configuration

#### Preliminaries

- Remove top panel and rear panel.
- Remove UPS and short the two pairs of wires with two connection blocks (+ with +, - with -)
- Install SSD and PCI-e to USB 3.0 cards inside the robot computer. Remove the dedicated graphics card to make room for the USB 3.0 card.
- Install Hokuyo UST-10LX LiDAR, RealSense D435i * 2, RealSense T265 * 1. Power the LiDAR from terminals (TBD).
- (NOT NECESSARY) ~~Change wiring harness (charging related) at battery terminal (behind battery bay): remove 06331**P** (Black), 06331**Q** (Black), 06021**A** (Red), 06021**C** (Red). Insulate separately and tie away.~~
- Install wireless charger. (TBD)
- Check and set servo motor IDs.

#### Setup Linux System

- Initialize Linux CAN bus (Needs to be done on every reboot):
```sh
sudo ip link set can0 up type can bitrate 1000000
```
- For debugging, use candump:
```sh
sudo apt-get install can-utils
candump -ax can0
```
The documentation for the iPOS motor controller CAN protocol can be found [HERE](http://www.technosoft.ro/KB/index.php?/getAttach/46/AA-15445/P091.063.CANopen.iPOS.UM.pdf).

- Alternatively, add the following lines to `/etc/rc.local` (**before** `exit 0`) to make system bring up `can0` and `can1` automatically during booting process:
```sh
sudo ip link set can0 up type can bitrate 1000000 triple-sampling on restart-ms 20
sudo ip link set can1 up type can bitrate 1000000 triple-sampling on restart-ms 20
# change congestion control method for CAN0 to ensure data packets are not stale under congestion.
sudo tc qdisc add dev can0 root handle 1: pfifo_head_drop limit 9
``` 

- ~~The robot has an LVDS built-in display interface for headless boot, and it is not used with an external display present. To disable it such that the external display becomes primary display, edit `/etc/default/grub` at **line 10** to be:~~
~~GRUB_CMDLINE_LINUX_DEFAULT="quiet splash video=LVDS-1:d"~~

**Note:** If the internal display is disabled, Ubuntu desktop may not boot properly without an external monitor plugged in. To reenable the internal display, revert the changes by deleting `video=...` section, and spesify the LVDS resolution in BIOS.

- This repository includes a bootup script, such that the SSH and VNC ports of the cart are mapped to a Virtual Private Server with static IP. To enable the automatic bootup sequence, add the following line in `crontab -e`:
```sh
@reboot sleep 30; cd /path_to_repo/; DISPLAY=:0 sh ./onBoot.sh
```
**WARNING:** The program will run 30s after the machine is booted up. Plugging in a joystick will mobilize the robot. Refer to `autorun.py` (under `src/pcv_base/scripts`) for series of commands it executes.

### System environments:

Ubuntu 16.04 or newer.

- Install prerequisites:
```sh
sudo apt-get update
sudo apt-get install apt-transport-https ca-certificates gnupg screen \
             software-properties-common wget doxygen openssh-server python-pip
```

- Install CMake 3.6+:
```sh
wget -qO - https://apt.kitware.com/keys/kitware-archive-latest.asc |
    sudo apt-key add -
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ xenial main'
sudo apt-get update
sudo apt-get install cmake
sudo apt-get install kitware-archive-keyring
sudo apt-key --keyring /etc/apt/trusted.gpg del C1F34CDD40CD72DA
```

- Install Intel librealsense:
```sh
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
```

- Install ROS: refer to online instructions ([Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) or [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)).

- Enable Desktop Sharing (from launcher find "Desktop Sharing")

- Optional: enable access of VNC through Windows machines (**NO sudo**):
```sh
gsettings set org.gnome.Vino require-encryption false
```

### Steps to run the code: 
```sh
sudo apt-get install ros-kinetic-amcl ros-kinetic-move-base ros-kinetic-gmapping \
              ros-kinetic-teb-local-planner ros-kinetic-dwa-local-planner ros-kinetic-urg-node \
              ros-kinetic-map-server ros-kinetic-realsense2-camera ros-kinetic-global-planner \
              ros-kinetic-eband-local-planner ros-kinetic-rtabmap ros-kinetic-ros-numpy \
              ros-kinetic-cv-bridge
sudo -H pip install numpy pyserial pymysql boto3 twillio opencv-contrib-python
catkin_make -DCMAKE_BUILD_TYPE=Release
```

Setup passwordless sudo for the current user: add the following line to `/etc/sudoers` **second line before ending**

```sh
username  ALL = (ALL) NOPASSWD: ALL
```
Then run the ros package `pcv_base`.

Automatic run procedure is defined in the autorun.py, which structures a move of the robot as a "task", as defined in the `pcv_base/scripts` folder:
```
autorun.py --(imports)--> task --(imports)--> payload
                            |----(calls)--> launch
                            |----(uses)--> resources
```
