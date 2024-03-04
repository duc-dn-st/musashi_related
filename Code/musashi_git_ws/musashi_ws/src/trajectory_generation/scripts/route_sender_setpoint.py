#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import time
from robot_state_manager.srv import RobotState

class RouteSender:

    def __init__(self):

        self.waypoint_pub = rospy.Publisher('/test_send_points', Float32MultiArray, queue_size=1)

        rospy.wait_for_service('robot_state_manager_service')

        self.robot_state_manager_service = rospy.ServiceProxy('robot_state_manager_service', RobotState)
        
        self.robot_state_manager_service("Waiting_For_Trajectory")

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

    def input_route(self):

        set_input = True
        set_route = True

        while set_route == True:

            while set_input == True:

                input_route = input("ルートの座標を入力してください : ")

                if self.checkRoute(input_route):

                    route = self.getRoute(input_route)
                    
                    set_input = False
                
                else:
                    rospy.logerr("入力ルートをもう一度確認してください")
                    
            temp_confirm = input(f"ルート : {route} - 確認する (y/n):")

            if temp_confirm == "y" or temp_confirm == "": 
                rospy.loginfo("ルートを確認しました")
                set_route = False

            else: 
                rospy.logerr("中止されました！ ルートを再度入力してください")
            
        return route

    # Publish waiypoints
    def send_waypoints(self, way_points):

        way_points.insert(0, float(len(way_points)))
        way_points_msg=Float32MultiArray(data=way_points)

        time.sleep(1)

        self.waypoint_pub.publish(way_points_msg)

if __name__ == '__main__':

    rospy.init_node('route_sender', anonymous=True)

    rs = RouteSender()

    while not rospy.is_shutdown():

        route = rs.input_route()    ## Recieve route No. 

        if route == 0:
            break
        else:
            rs.send_waypoints(route)     ## Send recieved way