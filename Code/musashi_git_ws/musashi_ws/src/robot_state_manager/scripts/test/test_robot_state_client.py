#! /usr/bin/env python3

import rospy

from robot_state_manager.srv import RobotState

if __name__ == "__main__":
    
    rospy.init_node("test_robot_state_client")
   
    rospy.loginfo("Starting test_robot_state_client.")

    robot_state_manager_service = rospy.ServiceProxy("robot_state_manager_service", RobotState)

    # ===========

    change_robot_state_req = "received_trajectory"
    
    change_robot_state_res = robot_state_manager_service(change_robot_state_req)

    print(f"{change_robot_state_req} → {change_robot_state_res}")

    # ===========

    get_robot_state_req = "GET"
    
    get_robot_state_res = robot_state_manager_service(get_robot_state_req)

    print(f"{get_robot_state_req} → {get_robot_state_res}")