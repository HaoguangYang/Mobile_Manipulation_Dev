# Mobile_Manipulation_Dev
Development repo for the mobile manipulation platforms

### Robot configuration

#### Preliminaries

- Remove top panel and rear panel.
- Remove UPS: (To be filled)
- Change wiring harness (charging related) at battery terminal (behind battery bay): remove 06331R (Black), 06331Q (Black), 06021B (Red), 06021C (Red). Insulate separately and tie away.
- Check main terminal box (DIN main) behind the switch panel, the wiring order should be as followed: (To be filled).
- Set servo motor IDs: (To be filled)

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

- Alternatively, add the following lines to `/etc/rc.local` (**before** `exit 0`) to make system bring up `can0` and `can1` automatically during booting process:
```sh
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
``` 

- The robot has an LVDS built-in display interface for some reason, and it is never used. To disable it such that the external display becomes primary display, edit `/etc/default/grub` at **line 10** to be:
```sh
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash video=LVDS-1:d"
```

### System environments:

Ubuntu 16.04 or newer.

- Install prerequisites:
```sh
sudo apt-get update
sudo apt-get install apt-transport-https ca-certificates gnupg software-properties-common wget doxygen
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
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
```

- Install ROS: refer to online instructions ([Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) or [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)).

### Steps to run the code: 
```sh
catkin_make
```

Setup passwordless sudo for the current user: add the following line to `/etc/sudoers` **second line before ending**

```sh
username  ALL = (ALL) NOPASSWD: ALL
```
Then run the ros package `pcv_base`.

