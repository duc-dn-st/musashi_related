#!/usr/bin/env python
import rospy
import numpy
from std_msgs.msg import Int16MultiArray
from kv_host_link import *

class RobotCommunication:
    def __init__(self, v_l, v_r, x, y, theta):
        self.v_l = v_l
        self.v_r = v_r
        self.x = x
        self.y = y
        self.theta = theta

    def callback_velocity_command(self,msg):
        rospy.loginfo("Message '{}' recieved".format(msg.data))
        self.v_l = msg.data[0]
        self.v_r = msg.data[1]
        # rospy.loginfo("v_l: '{}' ".format(self.v_l))
        # rospy.loginfo("VELOCITY_LEFT : {}".format(msg.data[0]))
        # rospy.loginfo("VELOCITY_RIGHT : {}".format(msg.data[1]))
        rc.write_robot()
        # rc.read_robot()


    def write_robot(self):
        if not kv.write_plc(VELOCITY_LEFT, self.v_l):
            rospy.loginfo("VL command sent.")
        else:
            rospy.logwarn("VL command sent error")

        if not kv.write_plc(VELOCITY_RIGHT, self.v_r):
            rospy.loginfo("VR command sent.")
        else:
            rospy.logwarn("VR command sent error")

    def read_robot(self):
        error, right_res = kv.read_plc(ENCODER_RIGHT)
        error, left_res = kv.read_plc(ENCODER_LEFT)
        rospy.loginfo("enc_right : " + str(right_res))
        rospy.loginfo("enc_left : " + str(left_res))

        right_res = right_res - prev_right

        prev_right = right_res
        prev_left = left_res

        if not error:
            right_encoder_data = right_res / 100
            left_encoder_data = left_res / 100

            v_r_enc = (right_encoder_data / 360) * ((2 * numpy.pi * 0.055) / 0.05)
            v_l_enc = (left_encoder_data / 360) * ((2 * numpy.pi * 0.055) / 0.05)

            v = (0.055 / 2) * (v_r_enc + v_l_enc)
            w = (0.055 / 0.33) * (v_r_enc - v_l_enc)

            self.x = self.x + (v * numpy.cos(self.theta)) * 0.05
            self.y = self.y + (v * numpy.sin(self.theta)) * 0.05
            self.theta = self.theta + w * 0.05

            rospy.loginfo("x : " + str(self.x))
            rospy.loginfo("y : " + str(self.y))
            rospy.loginfo("theta : " + str(self.theta))


    def robot_com(self):

        rospy.init_node('robot_communication', anonymous=True)

        rospy.Subscriber("/cmd_vel", Int16MultiArray, rc.callback_velocity_command)

        if not kv.connect_plc():
            rospy.loginfo("Connection to PLC successful.")

        else:
            rospy.logwarn("PLC connection error!")

        rospy.spin()

        # rate = rospy.Rate(20) # ROS Rate at 20Hz

        # while not rospy.is_shutdown():
        #     rc.write_robot()
        #     rate.sleep()
        self.v_l=0
        self.v_r=0
        rc.write_robot()
        kv.close_plc()


if __name__ == '__main__':
    kv = KvHostLink()
    rc = RobotCommunication(0, 0, 0.0, 0.0, 0.0)
    rc.robot_com()

