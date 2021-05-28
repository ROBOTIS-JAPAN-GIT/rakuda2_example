# Rakuda-2
![Rakuda2](https://user-images.githubusercontent.com/41886736/118063868-dd2bb580-b3d4-11eb-804c-80fe71bc2e1a.png)

## Quick Start

Please refer to the following for the ROS installation procedure.

[Ubuntu install of ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

```bash
$ mkdir ~/catkin_ws/src/ -p
$ cd ~/catkin_ws/src/

# Clone dependency packages
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone https://github.com/ROBOTIS-JAPAN-GIT/rakuda2_example.git

# Install package
$ sudo apt-get update
$ sudo apt-get install ros-$ROS_DISTRO-realsense2-camera

# Build and source
$ cd ~/catkin_ws && catkin_make
$ source ~/catkin_ws/devel/setup.bash

# Set Realsense serial number
# Connect realsense to your PC and enter the following command.
$ rs-enumerate-devices

# Check the serial number.
$ cd ~/catkin_ws/src/rakuda2_example/launch

# Enter the serial number you just checked in the default field.
$ nano rakuda2_master.launch
# <arg name="serial_no"             default=""/>

# Set DYNAMIXEL Port
# Connect U2D2 to your PC and enter the following command.
$ dmesg | tail -n 10
# [10975.907715] ftdi_sio 1-5:1.0: device disconnected
# [10981.622547] usb 1-5: new high-speed USB device number 9 using xhci_hcd
# [10981.775790] usb 1-5: New USB device found, idVendor=0403, idProduct=6014, bcdDevice= 9.00
# [10981.775795] usb 1-5: New USB device strings: Mfr=1, Product=2, SerialNumber=3
# [10981.775798] usb 1-5: Product: USB <-> Serial Converter
# [10981.775801] usb 1-5: Manufacturer: FTDI
# [10981.775803] usb 1-5: SerialNumber: FT3FSIF9
# [10981.780300] ftdi_sio 1-5:1.0: FTDI USB Serial Device converter detected
# [10981.780375] usb 1-5: Detected FT232H
# [10981.781315] usb 1-5: FTDI USB Serial Device converter now attached to ttyUSB0

# Enter the USB port you just checked in the default field.
$ nano rakuda2_master.launch
# <arg name="usb_port"     default="/dev/ttyUSB0"/>

# Copy rules file
$ cd ~/catkin_ws/src/rakuda2_example/
$ sudo cp ./99-rakuda2-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```

## demo
### When booting in master mode
```bash
roslaunch rakuda2_example rakuda2_master.launch
```

In master mode, it moves to the initial position, and after 3s, the torque of all the motors is disabled.
The data of each joint is published in the "/joint state".

### When booting in slave mode
```bash
roslaunch rakuda2_example rakuda2_slave.launch
```
In slave mode, it moves to the initial position, and the all motor torque is enabled.
The data obtained from the "/joint state" is written to each joint.


### Save to rosbag
```bash
rosbag recoard -a -O rakuda_data.bag
```

### Play from rosbag
```bash
rosbag play rakuda_data.bag
```

## ROBOTIS e-Manual for DynamixelSDK
[DynamixelSDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)

## ROBOTIS e-Manual for DYNAMIXEL X-Series
[XM430-W350](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)

[XM540-W270](https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/)

## ROBOTIS e-Manual for Communication Interface
[U2D2](https://emanual.robotis.com/docs/en/parts/interface/u2d2/)

[U2D2 Power Hub](https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/)

## e-Manual for IntelRealSense
[IntelRealSense](https://github.com/IntelRealSense/realsense-ros)
