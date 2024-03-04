#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import time

class RouteSender:

    def __init__(self):

        self.waypoint_pub = rospy.Publisher('/test_send_points', Float32MultiArray, queue_size=1)

    def input_route(self):
        
        set_route = True

        while set_route == True:

            temp_str = input("Please input route No. (0=quit):")
            
            try:
                route_no = int(temp_str)

                if route_no == 0:

                    return route_no

                temp_confirm = input(f"Confirm input route No. is {route_no} (y/n):")

                if temp_confirm == "y" or temp_confirm == "yes" : 
                    
                    print("###### Route confirmed ######")
                    
                    set_route = False
                
                else: 

                    print("######  Aborted. Please choose route No.again  ######")
                    
                    continue

            except ValueError:
                return 0

        return route_no

    # Publish waiypoints
    def send_waypoints(self, destination):

        print("###### Trajectory selecting mode #####")

        # Please chenge 'way_points' to the route you need.
        if destination == 1:
            way_points = [1.0, 1.0, 2.0, 1.0, 2.5, 1.5, 3.0, 1.5]
        elif destination == 2:
            way_points = [3.5, 1.0, 3.5, 4.0, 4.5, 4.0]
        
        way_points.insert(0, float(len(way_points)))     # [ length, x1, y1, x2, y2, ...]
        way_points_msg=Float32MultiArray(data=way_points)

        time.sleep(1)

        print("+++PRT+++ Published waypoint:\n%s" % way_points_msg)

        self.waypoint_pub.publish(way_points_msg)

        # rate = rospy.Rate(10) # 10hz

        # while not rospy.is_shutdown():
            
        #     self.waypoint_pub.publish(way_points_msg)
                        
        #     rate.sleep()

if __name__ == '__main__':

    print("###### Select Trajectory Mode #####")

    rospy.init_node('route_sender', anonymous=True)

    rs = RouteSender()

    while not rospy.is_shutdown():

        route_number=int( rs.input_route() )    ## Recieve route No. 

        if route_number == 0:
            break
        else:
            rs.send_waypoints(route_number)     ## Send recieved way
