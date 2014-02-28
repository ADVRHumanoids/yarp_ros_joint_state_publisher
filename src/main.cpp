#include "yarp_interface.h"
#include "ros_interface.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yarp_ros_joint_state_publisher");
    ros::NodeHandle n;

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

    ros_interface IRos;

    for(unsigned int i = 0; i < IRos.getKinematicChains().size(); ++i)
    {
        boost::shared_ptr<yarp_kinematic_chain> chain(new yarp_kinematic_chain(IRos.getKinematicChains()[i].first));
        IRos.addKinematicChain(chain);
    }


    /** The order is important! (MAYBE NOT!)
      1. torso
      2. r_leg
      3. l_leg
      4. r_arm
      5. l_arm              **/
    ROS_INFO("Beginning publishing joints state");
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        IRos.publish();
        loop_rate.sleep();
    }


	return 0;
}
