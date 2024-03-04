#include <ros/ros.h>
#include <robot_state_manager/RobotState.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_robot_state_client");

    ros::NodeHandle nh;

    ros::ServiceClient robot_state_client = nh.serviceClient<robot_state_manager::RobotState>("robot_state_manager_service");

    robot_state_manager::RobotState robot_state_srv;

    robot_state_srv.request.robot_state_request = "GET";

    if (robot_state_client.call(robot_state_srv))
    {
        std::cout << "Robot state : " <<robot_state_srv.response.robot_state_response << std::endl;
    }
    else
    {
        ROS_ERROR("Failed to call controller trigger service");

        return 1;

    }
        
    return 0;
}
