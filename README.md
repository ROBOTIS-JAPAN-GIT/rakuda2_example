# Rakuda-2
![Rakuda2](https://user-images.githubusercontent.com/41886736/118063868-dd2bb580-b3d4-11eb-804c-80fe71bc2e1a.png)

## Quick Start
```bash
mkdir ~/catkin_ws/src/ -p
cd ~/catkin_ws/src/

# Clone dependency packages
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone https://github.com/ROBOTIS-JAPAN-GIT/rakuda2_example.git

# Build and source
cd ~/catkin_ws && catkin_make
source ~/catkin_ws/devel/setup.bash
```

## demo
### When booting in master mode
```bash
roslaunch rakuda2_example rakuda2_master.launch
```
In master mode, the torque of all motors is disabled and the data of each joint is published in the "joint state".

### When booting in slave mode
```bash
roslaunch rakuda2_example rakuda2_slave.launch
```
In slave mode, all motor torque is enabled and the data obtained from the "joint state" is written to each joint.

## ROBOTIS e-Manual for DynamixelSDK
[DynamixelSDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)

## ROBOTIS e-Manual for DYNAMIXEL X-Series
[XM430-W350](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)

[XM540-W270](https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/)

## ROBOTIS e-Manual for Communication Interface
[U2D2](https://emanual.robotis.com/docs/en/parts/interface/u2d2/)

[U2D2 Power Hub](https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/)
