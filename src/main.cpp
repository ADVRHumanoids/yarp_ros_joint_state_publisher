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

    boost::shared_ptr<yarp_kinematic_chain> torso(new yarp_kinematic_chain("torso"));
    boost::shared_ptr<yarp_kinematic_chain> r_leg(new yarp_kinematic_chain("right_leg"));
    boost::shared_ptr<yarp_kinematic_chain> l_leg(new yarp_kinematic_chain("left_leg"));
    boost::shared_ptr<yarp_kinematic_chain> r_arm(new yarp_kinematic_chain("right_arm"));
    boost::shared_ptr<yarp_kinematic_chain> l_arm(new yarp_kinematic_chain("left_arm"));

    ros_interface IRos;
    /** The order is important!
      1. torso
      2. r_leg
      3. l_leg
      4. r_arm
      5. l_arm              **/
    IRos.addKinematicChain(torso);
    IRos.addKinematicChain(r_leg);
    IRos.addKinematicChain(l_leg);
    IRos.addKinematicChain(r_arm);
    IRos.addKinematicChain(l_arm);

    ROS_INFO("Beginning publishing joints state");
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        IRos.publish();
        loop_rate.sleep();
    }


	return 0;
}
