#! /usr/bin/env python3

import rospy

from std_msgs.msg import String

from robot_state_manager.srv import RobotState

class RobotStateManagerNode:

    # ==========================================================================
    # PUBLIC FUNCTION
    # ==========================================================================
    def __init__(self) -> None:
        
        rospy.init_node("robot_state_manager_node")

        self.robot_state = "Starting"

        self.robot_state_pub = rospy.Publisher("/robot_state_manager", String, queue_size=1)

        self.robot_state_msg = String()

        print(f"Returning {self.robot_state}")

        self._robot_state_manager_server()

    def run(self) -> None:

        while not rospy.is_shutdown():

            self.robot_state_msg.data = self.robot_state

            self.robot_state_pub.publish(self.robot_state_msg)

    # ==========================================================================
    # PRIVATE FUNCTION
    # ==========================================================================
    def _robot_state_manager_server(self):

        service = rospy.Service("robot_state_manager_service", RobotState, self._robot_state_manager_request)

    def _robot_state_manager_request(self, request) -> str:

        if request.robot_state_request == "GET":

            return self.robot_state
        
        else:

            self.robot_state = request.robot_state_request

            print(f"Returning {self.robot_state}")

            return  f"Returning {self.robot_state}"
