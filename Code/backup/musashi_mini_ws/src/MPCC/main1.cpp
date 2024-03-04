// EXPERIMENT


#include "Tests/spline_test.h"
#include "Tests/model_integrator_test.h"
#include "Tests/constratins_test.h"
#include "Tests/cost_test.h"

#include "MPC/mpc.h"
#include "Model/integrator.h"
#include "Params/track.h"
#include "Plotting/plotting.h"

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <time.h>
#include <ctime>

#include <nlohmann/json.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>

#include "../include/utilities/utilities.h"
#include <std_msgs/Int16MultiArray.h>

#define POS_THRESHOLD 0.3

using json = nlohmann::json;

nav_msgs::Path path_;
// nav_msgs::OdometryPtr robot_state;
geometry_msgs::PoseStamped::ConstPtr robot_state;
bool emergency_stop_flag_ = false;

void pathCallback(const nav_msgs::Path &path){

    std::cout << " pathCallback " << std::endl;
    path_ = path;
}
// void odometryCallback(const nav_msgs::OdometryPtr &odom_msg){

//     std::cout << " StateCallback " << std::endl;
//     robot_state = odom_msg;
// }

void odometryCallback(const geometry_msgs::PoseStamped::ConstPtr &odom_msg)
{
    // std::cout << " StateCallback " << std::endl;
    robot_state = odom_msg;
}

void emgFlagCallback(const std_msgs::Bool::ConstPtr& flag_msg)
{
    // ROS_INFO("I heard: [%s]", flag_msg->data ? "true" : "false");
    emergency_stop_flag_ = flag_msg->data;
}

bool atGoal(const Eigen::Vector2d &error)
{   

    if (std::abs(error(0)) < POS_THRESHOLD && std::abs(error(1)) < POS_THRESHOLD)
    {
        return true;
    }

    return false;
}

// void odomStateCallback(const nav_msgs::Odometry &data){

//     //std::cout << " rsStateCallback " << std::endl;
//     odometry_ = data;
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "model_predictive_contouring_controller");
    ros::NodeHandle priv_n("~");
    ros::NodeHandle nh;
    ros::Subscriber path_sub_ = nh.subscribe("/offline_trajectory_node/adapted_path", 1, pathCallback);
    // ros::Subscriber rs_sub_ = nh.subscribe("/R_001/robot_state", 1, rsStateCallback);
    ros::Subscriber odom_sub_ = nh.subscribe("/tracked_pose", 1, odometryCallback);
    ros::Subscriber flag_sub_ = nh.subscribe("/front_obstacle_detection_node/emg_stop_flag", 1, emgFlagCallback);

    using namespace mpcc;

    std::string const package_path = ros::package::getPath("model_predictive_contouring_control");
    std::ifstream iConfig(package_path + "/Params/config.json");
    json jsonConfig;
    iConfig >> jsonConfig;

    PathToJson json_paths {jsonConfig["model_path"],
                           jsonConfig["cost_path"],
                           jsonConfig["bounds_path"],
                           jsonConfig["track_path"],
                           jsonConfig["normalization_path"]};

    int return_flag;

    return_flag = testSpline();
    std::cout << " Result of testSpline(): " << return_flag << std::endl;

    return_flag = testArcLengthSpline(json_paths);
    std::cout << " Result of testArcLengthSpline(): " << return_flag << std::endl;

    return_flag = testIntegrator(json_paths);
    std::cout << " Result of testIntegrator(): " <<  return_flag << std::endl;

    return_flag = testLinModel(json_paths);
    std::cout << " Result of testLinModel(): " << return_flag << std::endl;
    //std::cout << testAlphaConstraint(json_paths) << std::endl;
    //std::cout << testTireForceConstraint(json_paths) << std::endl;

    return_flag = testTrackConstraint(json_paths);
    std::cout << " Result of testTrackConstraint(): " << return_flag << std::endl;

    return_flag = testCost(json_paths);
    std::cout << " Result of cost(): " << return_flag << std::endl;

    Integrator integrator = Integrator(jsonConfig["Ts"], json_paths);
    Plotting plotter = Plotting(jsonConfig["Ts"], json_paths);

    Track track = Track(json_paths.track_path);


    char filename_[30];
    time_t now = time(0);
    strftime(filename_, sizeof(filename_), "logs/%Y%m%d_%H%M.csv", localtime(&now));

    std::ofstream iniCSV;
    iniCSV.open(filename_, std::ios::out|std::ios::trunc);
    // iniCSV << "Horizon : " + std::to_string(ControlConstants::HORIZON) + ", Velocity" + std::to_string(TrajectoryParameters::PATH_VEL_LIM); 
    // iniCSV << std::endl;
    iniCSV <<   "s [m], pose_x [m], pose_y [m], pose_theta [rad], "
                "v [m/s], omega [rad/s], ";
                // "x_e, y_e, theta_e, x_ref, y_ref, theta_ref";
    iniCSV << std::endl;




    ros::Rate wait(1.0);
    while(path_.poses.size() == 0 && ros::ok()){
        std::cout << " waiting for path cb" << std::endl;
        ros::spinOnce();
        wait.sleep();
    }

    std::vector<double> x, y;

    for(int i = 0; i < path_.poses.size(); i++){
        x.push_back(path_.poses[i].pose.position.x);
        y.push_back(path_.poses[i].pose.position.y);
    }

    Eigen::Vector2d goal_point;
    goal_point << path_.poses[path_.poses.size()-1].pose.position.x, path_.poses[path_.poses.size()-1].pose.position.y;
    
    // double tolerence = 0.4;
    // double a = 0.0;
    // double b = 0.0 - tolerence;
 

    // while(a<=4.0 - tolerence)
    // {
    //     x.push_back(a);
    //     y.push_back(0);
    //     a = a + 0.1;
    // }

    // while(b>=-4.0)
    // {
    //     x.push_back(4.0);
    //     y.push_back(b);
    //     b = b - 0.1;
    // }

    // a = 10.0 + tolerence;
    // while(a<=5.0)
    // {
    //     x.push_back(a);
    //     y.push_back(5.0);
    //     a = a + 0.1;
    // }

    // x.push_back(0);
    // x.push_back(15.0);
    // x.push_back(15.0);

    // y.push_back(0);
    // y.push_back(0);
    // y.push_back(15.0);

    track.setTrack(x, y); // Set ROS Global Path

    TrackPos track_xy = track.getTrack();

    std::list<MPCReturn> log;
    MPC mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths);
    mpc.setTrack(track_xy.X, track_xy.Y);
    const double phi_0 = std::atan2(track_xy.Y(1) - track_xy.Y(0),track_xy.X(1) - track_xy.X(0));
    /*
    State x0 = {track_xy.X(0), track_xy.Y(0), phi_0, jsonConfig["v0"], 0,0};
    
    for(int i=0;i<jsonConfig["n_sim"];i++)
    {
        MPCReturn mpc_sol = mpc.runMPC(x0);
        x0 = integrator.simTimeStep(x0, mpc_sol.u0, jsonConfig["Ts"]);
        log.push_back(mpc_sol);
    }
    */

    // ROS
    ros::Publisher path_pub_ = priv_n.advertise<nav_msgs::Path>("predicted_path", 1);
    // ros::Publisher cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Publisher velocity_publisher_ = nh.advertise<std_msgs::Int16MultiArray>("cmd_vel", 10);
    nav_msgs::Path raw_path;
    geometry_msgs::Twist cmd_vel;
    std_msgs::Int16MultiArray velocity_msgs_;

    double ts = jsonConfig["Ts"];
    int i = 0;
    ros::Rate r(1.0f/ts);
    double lin_vel = 0.0;
    double prev_lin_vel = 0.0;
    double ang_vel = 0;
    double vs = 0;

    double yaw = tf::getYaw(robot_state->pose.orientation);
        if (yaw < 0)
        {yaw += 2.0 * M_PI;}

    State x0 = {robot_state->pose.position.x, robot_state->pose.position.y, yaw, 0, prev_lin_vel, 0};
    
    while(ros::ok())
    {
        raw_path.poses.clear();

        std::cout << "x0.X, Y, phi: " << x0.X << ", " << x0.Y << ", " << x0.phi << std::endl;
        std::cout << "x0.s, vx, vs: " << x0.s << ", " << x0.vx << ", " << x0.vs << std::endl;

        x0.X = robot_state->pose.position.x;
        x0.Y = robot_state->pose.position.y;

        Eigen::Vector2d current_robot_pose;
        current_robot_pose << robot_state->pose.position.x, robot_state->pose.position.y;
        Eigen::Vector2d error = goal_point - current_robot_pose;

        if(atGoal(error))
        {
            // Eigen::Vector2d input(0.0, 0.0);
            // rpm_convert_publish(input);
            break;
        }
        // else if(emergency_stop_flag_)
        // {
        //     prev_lin_vel = 0.0;
        //     vs = 0.0;

        //     velocity_msgs_.data.resize(2);
        //     //vl
        //     velocity_msgs_.data[0] = 0;
        //     //vr
        //     velocity_msgs_.data[1] = 0;

        //     velocity_publisher_.publish(velocity_msgs_);

        //     ros::spinOnce();
        //     r.sleep();
        // }
        else{
            double yaw = tf::getYaw(robot_state->pose.orientation);
            if (yaw < 0)
            {yaw += 2.0 * M_PI;}

            x0.phi = yaw;
            // x0.vx = robot_state->twist.twist.linear.x;
            x0.vx = prev_lin_vel;
            MPCReturn mpc_sol = mpc.runMPC(x0);

            lin_vel += mpc_sol.u0.dVx;
            vs += mpc_sol.u0.dVs;
            std::cout << "linear.x, angular.z: " << lin_vel << ", " << mpc_sol.u0.dPhi << std::endl;
            cmd_vel.linear.x = lin_vel;
            prev_lin_vel = lin_vel;
            cmd_vel.angular.z = mpc_sol.u0.dPhi;

            /////////////////TEMP/////////////////
            Eigen::Vector2d input(cmd_vel.linear.x, cmd_vel.angular.z);
            static double wheel_velocities [2];
            Eigen::Vector2d v_rpm;

            velocity_msgs_.data.resize(2);

            double angular_robot_vel = input(1);
            double linear_robot_vel = input(0);

            // if (std::abs(angular_robot_vel) < 0.001)
            // {
            //     double angular_robot_vel = 0.0;

            // }
            
            double left_wheel_vel = (linear_robot_vel / RobotConstants::WHEEL_RADIUS) - (RobotConstants::AXLE_LENGTH / 2 * angular_robot_vel / RobotConstants::WHEEL_RADIUS);
            wheel_velocities[0] = left_wheel_vel;

            double right_wheel_vel = (linear_robot_vel / RobotConstants::WHEEL_RADIUS) + (RobotConstants::AXLE_LENGTH / 2 * angular_robot_vel / RobotConstants::WHEEL_RADIUS);
            wheel_velocities[1] = right_wheel_vel;
            // left
            // v_rpm[0] = round((wheel_velocities[0] * 60/MathConstants::TWOPI) * 10) / 10;
            // value in *10
            v_rpm(0) = round((wheel_velocities[0] * 60/MathConstants::TWOPI) * 10);
            // right
            v_rpm(1) = round((wheel_velocities[1] * 60/MathConstants::TWOPI) * 10);

            //vl
            velocity_msgs_.data[0] = v_rpm(0);
            //vr
            velocity_msgs_.data[1] = v_rpm(1);

            velocity_publisher_.publish(velocity_msgs_);
            ////////////////////////////?TEMP///////////////////////////////

            // cmd_vel_pub_.publish(cmd_vel);

            x0 = integrator.simTimeStep(x0, mpc_sol.u0, ts);
            
            raw_path.header.frame_id = "odom";
            raw_path.header.stamp = ros::Time::now();
            for(int j=0;j<mpc_sol.mpc_horizon.size();j++)
            {
                geometry_msgs::PoseStamped track_pose;
                track_pose.pose.position.x = mpc_sol.mpc_horizon[0].xk.X;
                track_pose.pose.position.y = mpc_sol.mpc_horizon[0].xk.Y;
                track_pose.pose.orientation.z = 1;
                raw_path.poses.insert(raw_path.poses.end(), track_pose);
            }
            path_pub_.publish(raw_path); // For visualization

            i++;
            if(jsonConfig["n_sim"] < i)   //????????????
                break;

            log.push_back(mpc_sol);
            // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            
            ros::spinOnce();
            r.sleep();
        }
    }
    // geometry_msgs::Twist stop_vel;
    // cmd_vel_pub_.publish(stop_vel);
////////////////////////////////////TEMP/////////////
    //vl    
    velocity_msgs_.data[0] = 0.0;
    //vr
    velocity_msgs_.data[1] = 0.0;

    velocity_publisher_.publish(velocity_msgs_);
///////////////////////TEMP////////////////////////
    
    plotter.plotRun(log, track_xy);
    // plotter.plotSim(log, track_xy);

    double mean_time = 0.0;
    double max_time = 0.0;
    for(MPCReturn log_i : log)
    {
        mean_time += log_i.time_total;
        if(log_i.time_total > max_time)
            max_time = log_i.time_total;
    }
    std::cout << "mean nmpc time " << mean_time/double(jsonConfig["n_sim"]) << std::endl;
    std::cout << "max nmpc time " << max_time << std::endl;

    std::ofstream export_data;

    export_data.open(filename_, std::ios::out|std::ios::app);
    for(MPCReturn log_i : log)
    {
        export_data << log_i.mpc_horizon[0].xk.s << ", ";
        export_data << log_i.mpc_horizon[0].xk.X << ", ";
        export_data << log_i.mpc_horizon[0].xk.Y << ", ";
        export_data << log_i.mpc_horizon[0].xk.phi << ", ";
        export_data << log_i.mpc_horizon[0].xk.vx << ", ";
        export_data << log_i.mpc_horizon[0].uk.dPhi << ", ";
        export_data << std::endl;
        

        // plot_x.push_back(log_i.mpc_horizon[0].xk.X);
        // plot_y.push_back(log_i.mpc_horizon[0].xk.Y);
        // plot_phi.push_back(log_i.mpc_horizon[0].xk.phi);
        // plot_vx.push_back(log_i.mpc_horizon[0].xk.vx);
        // //plot_vy.push_back(log_i.mpc_horizon[0].xk.vy);
        // //plot_r.push_back(log_i.mpc_horizon[0].xk.r);
        // plot_s.push_back(log_i.mpc_horizon[0].xk.s);
        // //plot_d.push_back(log_i.mpc_horizon[0].xk.D);
        // //plot_delta.push_back(log_i.mpc_horizon[0].xk.delta);
        // plot_vs.push_back(log_i.mpc_horizon[0].xk.vs);

        // plot_dvx.push_back(log_i.mpc_horizon[0].uk.dVx);
        // plot_dphi.push_back(log_i.mpc_horizon[0].uk.dPhi);
        // plot_dvs.push_back(log_i.mpc_horizon[0].uk.dVs);
    }

    export_data << std::endl;

    return 0;
}