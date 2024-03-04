#!/usr/bin/env python3
from kv_host_link import *

import rospy
from std_msgs.msg import String, Float32MultiArray
from robot_state_manager.srv import RobotState

class RobotUserInterface:

    def __init__(self):
        
        # ===========   ROS   =============
        # ROS Node Init
        rospy.init_node('robot_user_interface', anonymous=True)

        # ROS Service
        self.robot_state_manager_service = rospy.ServiceProxy('robot_state_manager_service', RobotState)

        rospy.wait_for_service('robot_state_manager_service')

        self.robot_state_manager_service("Waiting_For_Trajectory")

        # ROS Subscriber
        self.test_sub = rospy.Subscriber('/test_topic', String, self.test_ui_callback)
        
        self.goal_msg_sub = rospy.Subscriber('/mai_goal_msgs_', String, self.goal_msg_callback)

        self.waypoint_pub = rospy.Publisher('/test_send_points', Float32MultiArray, queue_size=1)
        # ===================================
        
    # ===================================
    # PUBLIC FUNCTIONs
    # ===================================

    def run(self) -> None:
        """! Start ros node
        """
        rospy.spin

    # ===================================
    # PRIVATE FUNCTIONs
    # ===================================

    def test_ui_callback(self, msg):
        """! Get command from UI Test Node 
        Command type : String """

        # ===================================

        # Current : Getting data from /test_topic rostopic

        # 1. Change to get data from PLC        

        # ===================================
        
        self.command = msg.data
        rospy.loginfo(self.command)

        if self.command == '0' or self.command == 'reset':  # Reset Trajectory with Route = 0

            rospy.loginfo('### Reset_Trajectory ###')

            self.reset_trajectory()

        elif self.command == "run":                         # Start Controller when trajectory is set

            rospy.loginfo('### Start_Controller ###')

            self.start_controller()

        else:                                               # Get Route
            rospy.loginfo('### Receive Route ###')

            self.setRoute(self.command)

    def goal_msg_callback(self, goal_msg):

        if goal_msg == "AT_GOAL":

            print("Robot reached goal")

            # ===================================

            # 2. Need to add "Clear PLC Route at goal" here

            # ===================================
        else:
        
            pass
    
    def reset_trajectory(self):
        
        request = "GET"

        robot_state = self.robot_state_manager_service(request)

        if robot_state.robot_state_response != "Waiting_For_Trajectory":

            request = "Reset_Trajectory"

            response = self.robot_state_manager_service(request)

        else:

            rospy.logwarn("TRAJECTORY IS EMPTY")

    def start_controller(self):

        request = "GET"

        robot_state = self.robot_state_manager_service(request)

        if robot_state.robot_state_response == "Set_Trajectory":

            running_request = "Running"

            response = self.robot_state_manager_service(running_request)

        else:

            rospy.logwarn("YOU NEED TO SET TRAJECTORY FIRST")

    def removeSpace(self, route):

        return [i.strip() for i in route]

    def convertStrToFloat(self, route):
        try:

            return [float(i) for i in route]

        except:

            rospy.logerr("入力は数値である必要があります !!!!")

    # Check route correctness
    def checkRoute(self, input_route):

        route = input_route.split(",")

        route = self.removeSpace(route)

        route = self.convertStrToFloat(route)

        if route == None:

            return False
        
        if len(route)%2 == 0:

            return True
        else:

            rospy.logerr("入力量は偶数である必要があります !!!!!")

            return False
                 
    def getRoute(self, input_route):

        route = input_route.split(",")

        route = self.removeSpace(route)

        route = self.convertStrToFloat(route)

        return route
    
    # Publish waiypoints
    def sendWaypoints(self, way_points):

        way_points.insert(0, float(len(way_points)))

        way_points_msg=Float32MultiArray(data=way_points)

        time.sleep(1)

        self.waypoint_pub.publish(way_points_msg)

    def setRoute(self, input_route):

        request = "GET"

        robot_state = self.robot_state_manager_service(request)

        if robot_state.robot_state_response == "Waiting_For_Trajectory":

            feasible = self.checkRoute(input_route)

            if feasible == True:

                route = self.getRoute(input_route)

                self.sendWaypoints(route)
            
            else:

                rospy.loginfo("Not feasible")

        else:

            rospy.logwarn("YOU NEED TO STOP THE ROBOT FIRST")

if __name__ == '__main__':
    kv = KvHostLink()
    rc = RobotUserInterface()
    rospy.spin()

