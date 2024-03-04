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
#include "geometry_msgs/PoseStamped.h"


class Generate_trajectory_class
{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber route_sub_;
        ros::Subscriber tracked_pose_sub_;

        // message
        std_msgs::Float32MultiArray route_msg_;

        geometry_msgs::PoseStamped tracked_pose_msg_;

        double robot_current_tracked_pose[2];
        int test_route_size;
        double test_route[];

    public:
        Generate_trajectory_class(ros::NodeHandle nh, ros::NodeHandle private_nh);
        ~Generate_trajectory_class();

        void routeCallback(const std_msgs::Float32MultiArray::ConstPtr &received_route_msg);
        void trackPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &received_tracked_pose_msg);

};

Generate_trajectory_class::Generate_trajectory_class(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
    nh_ = nh;
    private_nh_ = private_nh;
    route_sub_ = nh_.subscribe("test_send_points", 1000, &Generate_trajectory_class::routeCallback, this);
    tracked_pose_sub_ = nh_.subscribe("tracked_pose", 1000, &Generate_trajectory_class::trackPoseCallback, this);
        
}
Generate_trajectory_class::~Generate_trajectory_class(){}

void Generate_trajectory_class::routeCallback(const std_msgs::Float32MultiArray::ConstPtr &received_route_msg)
{

    if (received_route_msg == NULL)
    {
        ROS_INFO("No messages received");
    }
    else
    {
        auto start_time = std::chrono::system_clock::now();

        route_msg_ = * received_route_msg;

        test_route_size = route_msg_.data[0];

        test_route[test_route_size] = {};

        for (size_t i = 0; i < test_route_size; i++)
        {
            test_route[i] = route_msg_.data[i+1];
        }

        // ROS_INFO("ROUTE messages received");

        // ------------------------------------------------


        Global_traj_class GlobalTraj(nh_, private_nh_);

        GlobalTraj.setRoute(test_route, test_route_size, robot_current_tracked_pose);

        Eigen::RowVector2d robot_current_position;      // [m] Robot's current [X, Y] coordinates in the global map
        // robot_current_position.setZero();
        robot_current_position << robot_current_tracked_pose[0] , robot_current_tracked_pose[1];

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
}

void Generate_trajectory_class::trackPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &received_tracked_pose_msg)
{
    tracked_pose_msg_ = * received_tracked_pose_msg;

    double pos_x = tracked_pose_msg_.pose.position.x;
    double pos_y = tracked_pose_msg_.pose.position.y;
    // double pos_z = tracked_pose_msg_.pose.position.z;

    // double orient_x = tracked_pose_msg_.pose.orientation.x;
    // double orient_y = tracked_pose_msg_.pose.orientation.y;
    // double orient_z = tracked_pose_msg_.pose.orientation.z;
    // double orient_w = tracked_pose_msg_.pose.orientation.w;

    robot_current_tracked_pose[0] = pos_x;
    robot_current_tracked_pose[1] = pos_y;

    // std::cout << "Postion : " << robot_current_tracked_pose[0] << " , " << robot_current_tracked_pose[1]<< std::endl;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offline_trajectory_node");

    ROS_INFO("  SET ROUTE MODE  ");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    Generate_trajectory_class generate_trajectory(nh, private_nh);

    ros::spin();

    return 0;
    
}
