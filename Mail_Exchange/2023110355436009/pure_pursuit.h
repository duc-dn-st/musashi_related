
#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#endif // PURE_PURSUIT_H

#include <ros/ros.h>

#include <eigen3/Eigen/Core>

#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

#include <cmath>
#include <algorithm>
#include <mutex>
#include <vector>
#include <string>
#include <fstream>
#include <time.h>
#include <ctime>

#include "sdv_msgs/Trajectory.h"
#include "sdv_msgs/TrajectoryPoint.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <boost/math/constants/constants.hpp>
#include "../include/utilities/utilities.h"
#include <std_msgs/Int16MultiArray.h>

#include <std_msgs/String.h>

#define POS_THRESHOLD 0.05
#define HEADING_THRESHOLD 30 * MathConstants::PI / 180
#define FIRST_HEADING_THRESHOLD 5 * MathConstants::PI / 180

#define POS_THRESHOLD2 1.5

#define POS_THRESHOLD3 0.15

class PurePursuit
{
public:
    PurePursuit() {}

    PurePursuit(ros::NodeHandle nh, ros::NodeHandle private_nh, double sampling_time, sdv_msgs::Trajectory &traj);

    void rotate(const Eigen::Vector3d &rear_axis_pose);

    void control(const Eigen::Vector3d &rear_axis_pose);

    void stopMotion(const Eigen::Vector3d &rear_axis_pose);

private:

    void generateCSV(const Eigen::Vector3d &pose, const Eigen::Vector2d &input);
    void read_reference();
    bool checkOrientation(const Eigen::Vector3d &error);
    bool atGoal(const Eigen::Vector3d &error) const;
    bool LastPoint(const Eigen::Vector3d &error) const;

    sdv_msgs::Trajectory traj_;

    /// calculate the current velocity with respect to epsilon ("curvature") of the path
    double getLinearVelocity(const Eigen::Vector3d &rear_axis_pose, const Eigen::Vector3d &vfh_goal);

    // for generating a trajectory model of the system needed
    inline void getStateUpdate(const Eigen::Vector3d &prev_state, const Eigen::Vector2d input, Eigen::Vector3d &state);

    // get the angle between robot heading and lookahead line
    inline double getAlpha(const Eigen::Vector3d &robot_pose, const Eigen::Vector2d &vfh_position);

    // returns the input from the vfh
    void getVfhControlCmd(Eigen::Vector2d &input);

    void rpm_convert_publish(Eigen::Vector2d &input);

    // Musashi AI
    void pubGoalStatus(void);

    // check linear acc constraint of ackerman drive for center wheel
    inline double linearAccConstraint(double lin_vel, double alpha)
    {
        double lin_acc = (lin_vel - prev_lin_vel_) / sampling_time_;
        // calculate max acc for center wheel
        double max_acc = RobotConstants::MAX_WHEEL_ACC * vfh_distance_ / (vfh_distance_ + 2 * RobotConstants::AXLE_LENGTH * std::abs(std::sin(alpha)));
        if (std::abs(lin_acc) > max_acc)
        {
            if (lin_acc < 0)
            {
                return prev_lin_vel_ - max_acc * sampling_time_;
            }
            else
            {
                return prev_lin_vel_ + max_acc * sampling_time_;
            }
        }
        return lin_vel;
    }

    // check linear velocity constraint of ackerman drive for center wheel
    inline double linearVelConstraint(double lin_vel, double alpha)
    {
        // calculate max linear velocity for the center wheel
        double max_vel = RobotConstants::MAX_WHEEL_VEL * vfh_distance_ / (vfh_distance_ + 2 * RobotConstants::AXLE_LENGTH * std::abs(std::sin(alpha)));
        if (std::abs(lin_vel) > max_vel)
        {
            if (lin_vel < 0)
            {
                return -1 * max_vel;
            }
            else
            {
                return max_vel;
            }
        }
        return lin_vel;
    }

    
    char filename_[30];
    
    int flag;

    double sampling_time_;

    double vfh_distance_;

    double prev_lin_vel_;

    bool rotation_mode;
    bool check_heading;

    Eigen::MatrixXd pose_ref_;
    unsigned long counter_;
    Eigen::MatrixXd ref_;

    Eigen::Vector3d goal_;
    Eigen::Vector3d vfh_goal;
    Eigen::Vector3d first_pos;
    nav_msgs::PathPtr vfh_heading_path;


    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher velo_pub_;
    ros::Timer periodic_timer_;

    ros::Publisher velocity_publisher_;
    geometry_msgs::Twist vel_msgs_;
    std_msgs::Int16MultiArray velocity_msgs_;

    ros::Publisher mai_goal_publisher_;
    std_msgs::String mai_goal_msgs_;
};