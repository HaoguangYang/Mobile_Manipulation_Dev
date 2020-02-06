# Mobile_Manipulation_Dev
Development repo for the mobile manipulation platforms

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

### Robot configuration

- Set servo motor IDs: (To be filled)

- Initialize Linux CAN bus:
```sh
sudo ip link set can0 up type can bitrate 1000000
```
- For debugging, use candump:
```sh
sudo apt-get install can-utils
candump -ax can0
```

### Steps to run the code: 
```sh
catkin_make
```
Then run the ros package `pcv_base`.

