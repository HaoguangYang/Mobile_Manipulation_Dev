# Mobile_Manipulation_Dev
Development repo for the mobile manipulation platforms

### System environments:

Ubuntu 16.04
Install CMake 3.6+:
```
wget -qO - https://apt.kitware.com/keys/kitware-archive-latest.asc |
    sudo apt-key add -
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ xenial main'
sudo apt-get update
sudo apt-get install apt-transport-https ca-certificates gnupg software-properties-common wget
sudo apt-get install cmake
sudo apt-get install kitware-archive-keyring
sudo apt-key --keyring /etc/apt/trusted.gpg del C1F34CDD40CD72DA
```

### Robot configuration

- Set servo motor IDs: (To be filled)

- Initialize Linux CAN bus:
```
sudo ip link set can0 up type can bitrate 1000000
```
- For debugging, use candump:
```
sudo apt-get install can-utils
candump -ax can0
```

### Steps to run the code: 
```
mkdir build
cd build
cmake .. && make 
```
Then run the executable in the bin folder: 
```
sudo ./vehicle
```
