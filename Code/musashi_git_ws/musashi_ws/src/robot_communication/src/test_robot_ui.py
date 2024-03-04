#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import String, Float32MultiArray
from robot_state_manager.srv import RobotState

class TestRobotUI:

    def __init__(self) -> None:
        
        # ===========   ROS   =============
        # ROS Node Init
        rospy.init_node('test_robot_user_interface', anonymous=True)

        # ROS Pub
        self.pub = rospy.Publisher('test_topic', String, queue_size=1)

    def send_command(self):

        self.command = input("Input command : \n")

        test_msg = String(data = self.command)

        connections = self.pub.get_num_connections()

        while not rospy.is_shutdown():

            if connections > 0:

                self.pub.publish(test_msg)

                break

if __name__ == '__main__':

    test_ui = TestRobotUI()

    while not rospy.is_shutdown():

        test_ui.send_command()
