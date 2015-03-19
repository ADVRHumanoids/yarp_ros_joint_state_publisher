#include "ros_interface.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yarp_ros_joint_state_publisher");
    ros::NodeHandle n("~");

    ros::Rate yarp_check_network_rate(1);
    yarp::os::Network yarp_network;
    while(ros::ok())
    {
        if(!yarp_network.checkNetwork()){
        ROS_INFO("Yarp Network not available... run a yarpserver...");
        yarp_check_network_rate.sleep();}
        else
            break;
    }
    ROS_INFO("Starting yarp_ros_joint_state_publisher node");

    std::string robot_name;
    n.getParam("robot_name", robot_name);
    if(robot_name.empty()){
        ROS_ERROR("robot_name param not provided!");
        return 0;}

    std::string urdf_path;
    n.getParam("urdf_path", urdf_path);
    if(urdf_path.empty()){
        ROS_ERROR("urdf_path param not provided!");
        return 0;}

    std::string srdf_path;
    n.getParam("srdf_path", srdf_path);
    if(srdf_path.empty()){
        ROS_ERROR("srdf_path param not provided!");
        return 0;}

    ros_interface IRos(robot_name, urdf_path, srdf_path);

    ROS_INFO("Beginning publishing joints state");
    double hz;
    n.param("rate", hz, 50.0);

    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        IRos.publish();

        ros::spinOnce();
        loop_rate.sleep();
    }


	return 0;
}
