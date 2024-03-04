#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import time
from robot_state_manager.srv import RobotState

class ResetTrajectory:

    def __init__(self):

        self.robot_state_manager_service = rospy.ServiceProxy('robot_state_manager_service', RobotState)

    def reset_trajectory(self):
        
        request = "Reset_Trajectory"

        response = self.robot_state_manager_service(request)

if __name__ == '__main__':

    print("###### Reset Trajectory #####")

    rospy.init_node('reset_trajectory', anonymous=True)

    rs = ResetTrajectory()

    rs.reset_trajectory()

    