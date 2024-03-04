#include "sdv_msgs/Trajectory.h"
#include "sdv_msgs/TrajectoryPoint.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>


class Publish_trajectory_class
{
    private:
        ros::NodeHandle nh_;
        ros::Publisher traj_pub_;
        ros::Publisher path_pub_;
        ros::Subscriber traj_sub_;
        ros::Subscriber path_sub_;

        sdv_msgs::Trajectory traj_pub_msg_;
        nav_msgs::Path path_pub_msg_;

    public:
        Publish_trajectory_class(ros::NodeHandle nh);
        ~Publish_trajectory_class();
        void trajectoryCallback(const sdv_msgs::Trajectory &traj_sub_msg_);
        void pathCallback(const nav_msgs::Path &path_sub_msg_);
        void publishPathandTrajectory();
};

Publish_trajectory_class::Publish_trajectory_class(ros::NodeHandle nh)
{

    nh_ = nh;
    traj_pub_ = nh_.advertise<sdv_msgs::Trajectory>("trajectory", 100);
    path_pub_ = nh_.advertise<nav_msgs::Path>("adapted_path", 100);

    traj_sub_ = nh_.subscribe("/offline_trajectory_node/trajectory", 10, &Publish_trajectory_class::trajectoryCallback, this);
    path_sub_ = nh_.subscribe("/offline_trajectory_node/adapted_path", 100, &Publish_trajectory_class::pathCallback, this);

}

Publish_trajectory_class::~Publish_trajectory_class()
{
}

void Publish_trajectory_class::trajectoryCallback(const sdv_msgs::Trajectory &traj_sub_msg_)
{

    // ROS_INFO("Callback Traj");
    
    traj_pub_msg_ = traj_sub_msg_;

}

void Publish_trajectory_class::pathCallback(const nav_msgs::Path &path_sub_msg_)
{

    // ROS_INFO("Callback Path");

    path_pub_msg_ = path_sub_msg_;

}

void Publish_trajectory_class::publishPathandTrajectory()
{

    // ROS_INFO("Publish Path");

    traj_pub_.publish(traj_pub_msg_);
    path_pub_.publish(path_pub_msg_);

}

//----------------------------------------------------------------------------------
int main(int argc, char **argv)
{

    ros::init(argc, argv, "publish_path_and_trajectory");
    ros::NodeHandle nh;


    Publish_trajectory_class publish_trajectory(nh);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        publish_trajectory.publishPathandTrajectory();
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;

}
