#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import time

class RouteSender:

    def __init__(self):

        self.waypoint_pub = rospy.Publisher('/test_send_points', Float32MultiArray, queue_size=1)

    def removeSpace(self, list):

        return [item.strip() for item in list]

    def input_route(self):

        set_route = True

        while set_route == True:

            input_route = input("Please input route (0=quit) :")

            try:

                if input_route == "0":
                    return input_route
                
                else:
                    route = self.removeSpace(input_route.split(","))

                    route = [float(i) for i in route]

                    if (len(route)%2 == 0):

                        temp_confirm = input(f"Input route : {route} - Confirm (y/n):")

                        if temp_confirm == "y" or temp_confirm == "yes" : 
                            
                            print("Route confirmed")
                            
                            set_route = False

                        else: 

                            print("Aborted! Please input route again.")
                            
                            continue

                    else:

                        print("The number of variables must be even! Please input route again.")

                        continue

            except ValueError:
                return 0
            
        return route

   # Publish waiypoints
    def send_waypoints(self, way_points):

        way_points.insert(0, float(len(way_points)))
        way_points_msg=Float32MultiArray(data=way_points)

        time.sleep(1)

        # print("\n +++PRT+++ Published waypoint:\n%s" % way_points_msg)

        self.waypoint_pub.publish(way_points_msg)

if __name__ == '__main__':

    print("###### Select Trajectory Mode #####")

    rospy.init_node('route_sender', anonymous=True)

    rs = RouteSender()

    while not rospy.is_shutdown():

        route = rs.input_route()    ## Recieve route No. 

        if route == 0:
            break
        else:
            rs.send_waypoints(route)     ## Send recieved way