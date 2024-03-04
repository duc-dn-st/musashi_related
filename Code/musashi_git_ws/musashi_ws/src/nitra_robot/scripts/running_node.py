#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import time
from robot_state_manager.srv import RobotState

class RunningNode:

    def __init__(self):

        self.robot_state_manager_service = rospy.ServiceProxy('robot_state_manager_service', RobotState)

    def Running(self):
        
        request = "GET"

        robot_state = self.robot_state_manager_service(request)

        # print(robot_state.robot_state_response)

        if robot_state.robot_state_response == "Set_Trajectory":

            running_request = "Running"

            response = self.robot_state_manager_service(running_request)

        else:

            rospy.logwarn("YOU NEED TO SET TRAJECTORY FIRST")

if __name__ == '__main__':

    print("###### Controller Trigger #####")

    rospy.init_node('running_node', anonymous=True)

    rs = RunningNode()

    rs.Running()

    