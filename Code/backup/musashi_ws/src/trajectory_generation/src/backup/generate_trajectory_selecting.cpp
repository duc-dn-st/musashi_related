// Musashi_project_cpp.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "../include/trajectory_generation/global_trajectory.h"
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <cmath>
#include <math.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <chrono>

#include <ros/ros.h>

#include "sdv_msgs/Trajectory.h"
#include "sdv_msgs/TrajectoryPoint.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offline_trajectory_node");

    auto start_time = std::chrono::system_clock::now();

    ROS_INFO("  SET ROUTE MODE  ");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    std_msgs::Float32MultiArray test_route_msg;
    std_msgs::Float32MultiArray::ConstPtr retrieve_route_msg; 
    retrieve_route_msg = ros::topic::waitForMessage<std_msgs::Float32MultiArray>("/test_send_points");

    if (retrieve_route_msg == NULL)
    {
        ROS_INFO("No messages received");
    }
    else
    {
        test_route_msg = * retrieve_route_msg;

        int test_route_size = test_route_msg.data[0];

        double test_route[test_route_size] = {};

        for (size_t i = 0; i < test_route_size; i++)
        {
            test_route[i] = test_route_msg.data[i+1];
        }

        // ----------------------------------------------------

        Global_traj_class GlobalTraj(nh, private_nh);

        GlobalTraj.setRoute(test_route, test_route_size);

        Eigen::RowVector2d robot_current_position;      // [m] Robot's current [X, Y] coordinates in the global map
        robot_current_position.setZero();
        double robot_current_path_velocity = 0;         // [m/s] Robot's current path velocity

        GlobalTraj.get_robot_current_state(robot_current_position, robot_current_path_velocity);

        GlobalTraj.generateTrajectory();
        // GlobalTraj.export_global_trajectory();
        // GlobalTraj.rpm_convert();

        auto end_time = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_time = end_time - start_time;

        std::cout << "The global trajectory was successfully generated.\n";
        std::cout << "Elapsed time: " << 1000*elapsed_time.count() << " [msec].\n" << std::endl;

        ros::Rate loop_rate(1);

        // while (ros::ok())
        // {
        //     ros::spinOnce();
        //     GlobalTraj.publishPathAndTrajectory();
        //     loop_rate.sleep();
        // }
        
        ros::Time beginTime = ros::Time::now();
        ros::Duration secondsIWantToSendMessagesFor = ros::Duration(3); 
        ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;

        while(ros::Time::now() < endTime )
        {
            ros::spinOnce();
            GlobalTraj.publishPathAndTrajectory();
            loop_rate.sleep();
        }

    }

    // ros::spin();
        
    return 0;
}
