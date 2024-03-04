#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <eigen3/Eigen/Core>
#include <cmath>
#include <mutex>
#include "../include/pure_pursuit.h"
#include <robot_state_manager/RobotState.h>

class ControllerNode
{
public:
    ControllerNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
    {
        init(nh, private_nh);
        std::cout << "Init succesfull" << std::endl;
        initController();
        std::cout << "Controller init succesfull" << std::endl;
        // ros::Duration(1).sleep();
        action_timer_ = private_nh_.createTimer(ros::Duration(sampling_time_), &ControllerNode::controllerActionManager, this);
    }

    void trajectoryCallback(const sdv_msgs::Trajectory &traj_msg)
    {
        // std::lock_guard<std::mutex> lock_odom(odom_mutex_);
        double now = ros::Time::now().toSec();
        // store trajectory
        traj_msg_ = traj_msg;
    }

    void odometryCallback(const geometry_msgs::PoseStamped::ConstPtr &odom_msg)
    {
        std::lock_guard<std::mutex> lock_odom(odom_mutex_);
        double now = ros::Time::now().toSec();
        // store robot pose
        odom_msg_ = odom_msg;
    }

    void emgFlagCallback(const std_msgs::Bool::ConstPtr &flag_msg)
    {
        std::lock_guard<std::mutex> lock_flag(flag_mutex_);
        // store emergency flag
        emergency_stop_flag_ = flag_msg->data;
    }

    void robotStateCallback(const std_msgs::String::ConstPtr &robot_state_msg)
    {
        std::lock_guard<std::mutex> lock_flag(flag_mutex_);
        // std::cout << "I heard : "<< robot_state_msg->data << std::endl;
        // store robot state
        robot_state_msg_ = robot_state_msg;
    }

    // void callController(const ros::TimerEvent &event)
    void callController()
    {
        double now = ros::Time::now().toSec();
        last_ = now;

        // check if ok to execute
        if (odom_msg_ == nullptr)
        {
            ROS_WARN("CONTROLLER_NODE_WARN: No messages received from odom or encoder.");
            return;
        }

        // take data
        // pose
        double yaw = tf::getYaw(odom_msg_->pose.orientation);
        if (yaw < 0)
        {yaw += 2.0 * M_PI;}
        robot_pose_ << odom_msg_->pose.position.x, odom_msg_->pose.position.y, yaw;
        odom_mutex_.unlock();

        // EMG STOP
        if (emergency_stop_flag_)
        {
            // std::cout << "Stop Robot" << std::endl;
            // send stop signal to controller

            pure_pursuit_controller_.stopMotion(robot_pose_);
             // ROS_WARN("EMG STOP ACTIVATED");
        }
        else
        {
            pure_pursuit_controller_.control(robot_pose_);
        }
        flag_mutex_.unlock();
    }

private:
    ros::NodeHandle nh_, private_nh_;
    // subscriber to robot localization and encoder data
    ros::Subscriber traj_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber flag_sub_;
    ros::Subscriber robot_state_sub;
    ros::ServiceClient robot_state_client;
    ros::Publisher mai_goal_publisher_;

    // vector of robot pose (x, y, theta)
    // messages
    geometry_msgs::PoseStamped::ConstPtr odom_msg_;
    nav_msgs::PathPtr vfh_path_;
    sdv_msgs::Trajectory traj_msg_;
    std_msgs::String::ConstPtr robot_state_msg_;
    std_msgs::String mai_goal_msgs_;

    robot_state_manager::RobotState robot_state_srv;

    bool simulation_;
    bool emergency_stop_flag_;
    bool reached_goal_flag_;
    std::string robot_state_;

    // steer is in rad and wheel_vel is in m/s
    Eigen::Vector3d robot_pose_;

    // sampling time of the node
    double sampling_time_;
    ros::Timer periodic_timer_;
    ros::Timer action_timer_;
    // mutex for all variables which are affected of the multiple threads
    std::mutex odom_mutex_, encoder_mutex_, flag_mutex_, collision_mutex_;
    // object of the controller
    PurePursuit pure_pursuit_controller_;

    int update_counter_;

    // timer
    double last_;

    void init(ros::NodeHandle nh, ros::NodeHandle private_nh)
    {
        nh_ = nh;
        private_nh_ = private_nh;
        emergency_stop_flag_ = false;
        reached_goal_flag_ = false;
        // check parameter server
        nh.param("controller_sampling_time", sampling_time_, 0.05); // [s]
        
        // subscriber
        ROS_WARN("CONTROLLER_NODE_WARN: Subscribe 'odom' and 'obstacle detections' conditions");
        traj_sub_ = nh_.subscribe("/trajectory", 1, &ControllerNode::trajectoryCallback, this);
        odom_sub_ = nh_.subscribe("/tracked_pose", 4, &ControllerNode::odometryCallback, this);
        flag_sub_ = nh_.subscribe("/front_obstacle_detection_node/emg_stop_flag", 1, &ControllerNode::emgFlagCallback, this);
        robot_state_client = nh_.serviceClient<robot_state_manager::RobotState>("robot_state_manager_service");
        // periodic_timer_ = private_nh_.createTimer(ros::Duration(sampling_time_), &ControllerNode::callController, this);

    }

    void initController()
    {
        pure_pursuit_controller_ = PurePursuit(nh_, private_nh_, sampling_time_);
    }

    void controllerActionManager(const ros::TimerEvent &event)
    {
        robot_state_srv.request.robot_state_request = "GET";
        if (robot_state_client.call(robot_state_srv))
        {
            robot_state_ = robot_state_srv.response.robot_state_response;

            std::cout << "Robot State : " << robot_state_ << std::endl;
        }
        else
        {
            ROS_ERROR("Failed to call controller trigger service");
        }

        if (robot_state_ == "Waiting_For_Trajectory")
        {
            if(traj_msg_.points.size() > 50 )
            {
                robot_state_srv.request.robot_state_request = "Set_Trajectory";
                robot_state_client.call(robot_state_srv);
            }
            else if (reached_goal_flag_ == false && traj_msg_.points.size() < 50)
            {
                std::cout << "No trajectory ..." << std::endl;
                double yaw = tf::getYaw(odom_msg_->pose.orientation);
                if (yaw < 0)
                {yaw += 2.0 * M_PI;}
                robot_pose_ << odom_msg_->pose.position.x, odom_msg_->pose.position.y, yaw;
                pure_pursuit_controller_.stopMotion(robot_pose_);
            }
            else if (reached_goal_flag_ == true && traj_msg_.points.size() < 50)
            {
                std::cout << "Reached goal - Waiting for new trajectory ... " << std::endl;
                double yaw = tf::getYaw(odom_msg_->pose.orientation);
                if (yaw < 0)
                {yaw += 2.0 * M_PI;}
                robot_pose_ << odom_msg_->pose.position.x, odom_msg_->pose.position.y, yaw;
                pure_pursuit_controller_.stopMotion(robot_pose_);
                pubGoalStatus();
            }
        }
        else if (robot_state_ == "Set_Trajectory")
        {
            reached_goal_flag_ = false;
            pure_pursuit_controller_.setTrajectory(traj_msg_);
            std::cout << "Trajectory set - Waiting for controller to start ..." << std::endl;
            ros::Duration(1).sleep();
        }
        else if (robot_state_ == "Running")
        {
            callController();
            std::cout << "Following Trajectory ...  " << std::endl;
        }
        else if (robot_state_ == "Reached_goal")
        {
            reached_goal_flag_ = true;
            pure_pursuit_controller_.stopMotion(robot_pose_);
            robot_state_srv.request.robot_state_request = "Reset_Trajectory";
            robot_state_client.call(robot_state_srv);            
        }
        else if (robot_state_ == "Reset_Trajectory")
        {
            // ros::Duration(1).sleep();
            pure_pursuit_controller_.stopMotion(robot_pose_);
            pure_pursuit_controller_.setTrajectory(traj_msg_);
            
            std::cout << "Reached goal - Waiting for new trajectory ... " << std::endl;

            //-------------------------------------------
            pure_pursuit_controller_.flag = 0;
            pure_pursuit_controller_.rotation_mode = true;
            pure_pursuit_controller_.counter_ = 1;
            pure_pursuit_controller_.prev_lin_vel_ = 0;
            //-------------------------------------------
            robot_state_srv.request.robot_state_request = "Reseting_Trajectory";
            robot_state_client.call(robot_state_srv);    
        }
        
    }

    void pubGoalStatus(void)       // Musashi AI 
    {
        mai_goal_publisher_ = nh_.advertise<std_msgs::String >("mai_goal_msgs_", 1000);
        mai_goal_msgs_.data = "AT_GOAL";
        mai_goal_publisher_.publish(mai_goal_msgs_);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ControllerNode controller(nh, private_nh);

    ros::AsyncSpinner s(4);
    s.start();
    ros::waitForShutdown();
}
