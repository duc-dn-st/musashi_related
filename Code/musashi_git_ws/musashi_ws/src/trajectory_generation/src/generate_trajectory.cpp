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
#include <robot_state_manager/RobotState.h>
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
        ros::ServiceClient robot_state_client;
        ros::Timer action_timer_;

        // message
        std_msgs::Float32MultiArray route_msg_;
        geometry_msgs::PoseStamped tracked_pose_msg_;
        // Service
        robot_state_manager::RobotState robot_state_srv;

        std::string robot_state_;
        double robot_current_tracked_pose[2];
        int route_size;
        double reset_route[2];
        double route[];

        void actionManager(const ros::TimerEvent &event);
    public:
        Generate_trajectory_class(ros::NodeHandle nh, ros::NodeHandle private_nh);
        ~Generate_trajectory_class();

        void routeCallback(const std_msgs::Float32MultiArray::ConstPtr &received_route_msg);
        void trajectoryGeneration();
        void trackPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &received_tracked_pose_msg);
        void trajectoryReset();
};

Generate_trajectory_class::Generate_trajectory_class(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
    nh_ = nh;
    private_nh_ = private_nh;
    route_sub_ = nh_.subscribe("test_send_points", 1000, &Generate_trajectory_class::routeCallback, this);
    tracked_pose_sub_ = nh_.subscribe("tracked_pose", 1000, &Generate_trajectory_class::trackPoseCallback, this);
    robot_state_client = nh_.serviceClient<robot_state_manager::RobotState>("robot_state_manager_service");
    action_timer_ = private_nh_.createTimer(ros::Duration(0.05), &Generate_trajectory_class::actionManager, this);
}
Generate_trajectory_class::~Generate_trajectory_class(){}

void Generate_trajectory_class::routeCallback(const std_msgs::Float32MultiArray::ConstPtr &received_route_msg)
{
    if (received_route_msg != NULL)
    {
        route_msg_ = *received_route_msg;

        route_size = route_msg_.data[0];

        route[route_size] = {};

        for (size_t i = 0; i < route_size; i++)
        {
            route[i] = route_msg_.data[i+1];
        }
        trajectoryGeneration();
    }
    else
    {
        ROS_DEBUG("No route received...");
    }

}

void Generate_trajectory_class::trajectoryGeneration()
{

    auto start_time = std::chrono::system_clock::now();

    Global_traj_class GlobalTraj(nh_, private_nh_);

    GlobalTraj.setRoute(route, route_size, robot_current_tracked_pose);

    Eigen::RowVector2d robot_current_position;      // [m] Robot's current [X, Y] coordinates in the global map

    robot_current_position << robot_current_tracked_pose[0] , robot_current_tracked_pose[1];

    double robot_current_path_velocity = 0;         // [m/s] Robot's current path velocity

    GlobalTraj.get_robot_current_state(robot_current_position, robot_current_path_velocity);

    GlobalTraj.generateTrajectory();

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

void Generate_trajectory_class::actionManager(const ros::TimerEvent &event)
{
    robot_state_srv.request.robot_state_request = "GET";
    if (robot_state_client.call(robot_state_srv))
    {
        robot_state_ = robot_state_srv.response.robot_state_response;
        // std::cout << robot_state_ <<std::endl;
    }

    if (robot_state_== "Reset_Trajectory")
    {
        reset_route[0] = robot_current_tracked_pose[0];
        reset_route[1] = robot_current_tracked_pose[1];            
        Global_traj_class GlobalTraj(nh_, private_nh_);
        GlobalTraj.setRoute(reset_route, 2, robot_current_tracked_pose);
        Eigen::RowVector2d robot_current_position;
        robot_current_position << robot_current_tracked_pose[0] , robot_current_tracked_pose[1];

        double robot_current_path_velocity = 0;         // [m/s] Robot's current path velocity

        GlobalTraj.get_robot_current_state(robot_current_position, robot_current_path_velocity);
        GlobalTraj.generateTrajectory();

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
        robot_state_srv.request.robot_state_request = "Waiting_For_Trajectory";
        robot_state_client.call(robot_state_srv);

    }
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
