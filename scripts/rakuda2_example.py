#!/usr/bin/env python
# -*- coding: utf-8 -*-

#*******************************************************************************
# Copyright 2021 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#******************************************************************************/
# Authors: Takashi Kitajima

import rospy
import rosparam
import math
from sensor_msgs.msg import JointState
from dynamixel_sdk import *

PROTOCOL_VERSION            = 2.0 # DYNAMIXEL Protocol version
 
MIN_POS_VALUE    = 0              # X-Series min position
CENTOR_POS_VALUE = 2048           # X-Series center position
MAX_POS_VALUE    = 4095           # X-Series max position

ADDR_TORQUE_ENABLE          = 64  # X-Series torque enable address
ADDR_GOAL_POSITION          = 116 # X-Series goal position address
ADDR_PRESENT_POSITION       = 132 # X-Series present position address

LEN_GOAL_POSITION           = 4   # X-Series goal position size
LEN_PRESENT_POSITION        = 4   # X-Series present position size

TORQUE_ENABLE               = 1   # Torque enable value
TORQUE_DISABLE              = 0   # Torque disable value
NAME                        = 0   # Index of array joint names
ID                          = 1   # Index of array joint ids
PERIOD                      = 100 # 100Hz

# List of joint names and IDs
JOINTS_LIST = [
    ["torso_yaw",       27]
    #["head_yaw",        28]
    #["head_pitch",      29],
    #["r_arm_sh_pitch1", 1],
    #["r_arm_sh_roll",   3],
    #["r_arm_sh_pitch2", 5],
    #["r_arm_el_yaw",    7],
    #["r_arm_wr_roll",   9],
    #["r_arm_wr_yaw",    11],
    #["r_arm_grip",      31],
    #["l_arm_sh_pitch1", 2],
    #["l_arm_sh_roll",   4],
    #["l_arm_sh_pitch2", 6],
    #["l_arm_el_yaw",    8],
    #["l_arm_wr_roll",   10],
    #["l_arm_wr_yaw",    12],
    #["l_arm_grip",      30]
    ]

class Dynamixel:
    def __init__(self, mode):
        # Handler initialization
        self.__port_h = PortHandler(rospy.get_param("~usb_port"))
        self.__packet_h = PacketHandler(PROTOCOL_VERSION)
        self.__sync_write = GroupSyncWrite(self.__port_h, self.__packet_h, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
        self.__sync_read = GroupSyncRead(self.__port_h, self.__packet_h, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

        # Open communication port
        if self.__port_h.openPort():
            rospy.loginfo("Succeeded to open the port")
        else:
            rospy.loginfo("Failed to open the port")

        # Communication baud rate setting
        baud_rate = rospy.get_param("~baud_rate")
        if self.__port_h.setBaudRate(baud_rate):
            rospy.loginfo("Succeeded to change the baudrate")
        else:
            rospy.loginfo("Failed to change the baudrate")
        
        # Select the operation mode
        if mode == "master":
            torque = TORQUE_DISABLE
        elif mode == "slave":
            torque = TORQUE_ENABLE

        # Enable/Disable the torque of each joint
        for joint in JOINTS_LIST:
            self.__packet_h.write1ByteTxRx(self.__port_h, joint[ID], ADDR_TORQUE_ENABLE, torque)
            # Set to read the angle of each joint in synchronization
            result = self.__sync_read.addParam(joint[ID])
            if result != True:
                rospy.logerr("ID:%03d groupSyncRead addparam failed" % joint[ID])

    def send_read_command(self):
        # Send a synchronous read command
        comm_e = self.__sync_read.txRxPacket()
        if comm_e == COMM_SUCCESS:
            return True
        else:
            rospy.logerr("%s" % self.__packet_h.getTxRxResult(comm_e))
            return False

    def read_position(self, id):
        # Confirm the acquisition of DYNAMIXEL data corresponding to the ID
        result = self.__sync_read.isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if result == True:
            # Get the DYNAMIXEL data corresponding to the ID
            pos = self.conv_pos_to_radian(self.__sync_read.getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION))
            if pos >= math.pi :
                pos = math.pi
            elif pos <= -math.pi:
                pos = -math.pi
        else:
            rospy.logerr("[ID:%03d] groupSyncRead getdata failed" % id)
        return pos

    def send_write_command(self):
        # Send a synchronous write command
        comm_e = self.__sync_write.txPacket()
        if comm_e != COMM_SUCCESS:
            rospy.logerr("%s" % self.__packet_h.getTxRxResult(comm_e))
        
        # Clear the ID list of the added DYNAMIXEL
        self.__sync_write.clearParam()

    def write_position(self, id, pos):
        # Converts the angle value to the command value of DYNAMIXEL
        value = self.conv_radian_to_pos(pos)
        param_goal_pos = [DXL_LOBYTE(DXL_LOWORD(value)), DXL_HIBYTE(DXL_LOWORD(value)), DXL_LOBYTE(DXL_HIWORD(value)), DXL_HIBYTE(DXL_HIWORD(value))]

        # Add a value to the list of synchronous writes
        result = self.__sync_write.addParam(int(id), param_goal_pos)
        if result != True:
            rospy.logerr("[ID:%03d] groupSyncWrite addparam failed" % id)

    def conv_radian_to_pos(self, radian):
        # Convert from radians to position command values
        value = MIN_POS_VALUE
        zero_pos = (MAX_POS_VALUE + MIN_POS_VALUE) / 2
        if radian > MIN_POS_VALUE:
            value = (radian * (MAX_POS_VALUE - zero_pos) / math.pi) + zero_pos
        elif radian < MIN_POS_VALUE:
            value = (radian * (MIN_POS_VALUE - zero_pos) / -math.pi) + zero_pos
        else:
            value = CENTOR_POS_VALUE
        return int(value)

    def conv_pos_to_radian(self, value):
        # Convert from position command values to radians
        radian = 0.0
        if value > CENTOR_POS_VALUE:
            radian = (value - CENTOR_POS_VALUE) * math.pi / (MAX_POS_VALUE - CENTOR_POS_VALUE)
        elif value < CENTOR_POS_VALUE:
            radian = (value - CENTOR_POS_VALUE) * -math.pi / (MIN_POS_VALUE - CENTOR_POS_VALUE)
        else:
            radian = 0.0
        return radian

def joint_states_callback(data):
    pos_list = []

    # Set the received joint angle data to DYNAMIXEL
    for joint, pos in zip(JOINTS_LIST, data.position):
        dynamixel.write_position(joint[ID], pos)
    dynamixel.send_write_command()

def main(mode):
    # In master mode, read and send DYNAMIXEL joint data
    if mode == "master":
        # Creating a joint states publisher
        joint_pub = rospy.Publisher('joint_states', JointState, queue_size=1)

        # Setting the operation period
        rate = rospy.Rate(PERIOD)

        # Loop until the node shuts down
        while not rospy.is_shutdown():

            # Create joint state
            joint_data = JointState()
            joint_data.header.stamp = rospy.Time.now()
            # Check of reading DYNAMIXEL angle data
            if dynamixel.send_read_command() == True:
                # Set joint name and joint angle
                for joint in JOINTS_LIST:
                    joint_data.name.append(joint[NAME])
                    joint_data.position.append(dynamixel.read_position(joint[ID]))

            # Send joint states
            joint_pub.publish(joint_data)
            rate.sleep()
    # In slave mode, write and recv DYNAMIXEL joint data
    elif mode == "slave":
        # Creating a joint states subscriber
        joint_sub = rospy.Subscriber('joint_states', JointState, joint_states_callback)
        rospy.spin()
    else:
        rospy.loginfo("Set mode to master or slave")

if __name__ == '__main__':
    try:
        # Initialize the node
        rospy.init_node('rakuda2_exaple', anonymous=True)

        # Get the operating mode
        mode = rospy.get_param("~mode")

        # Initialize DYNAMIXEL and execute main processing
        dynamixel = Dynamixel(mode)
        main(mode)
    except rospy.ROSInterruptException:
        pass
