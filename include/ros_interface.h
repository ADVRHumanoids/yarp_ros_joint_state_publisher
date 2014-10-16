#ifndef _ROS_INTERFACE_H_
#define _ROS_INTERFACE_H_

#include <sensor_msgs/JointState.h>
//#include "yarp_interface.h"
#include <drc_shared/idynutils.h>
#include <boost/shared_ptr.hpp>
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <srdfdom/model.h>
#include <urdf/model.h>
#include <drc_shared/yarp_single_chain_interface.h>
class ros_interface
{
public:
    ros_interface();
    ~ros_interface(){}

    void publish();

private:
    //yarp data is here, the others map use pointers
    std::vector<walkman::drc::yarp_single_chain_interface*> _kinematic_chains;
    ros::NodeHandle _n;
    ros::Publisher _joint_state_pub;
    urdf::Model coman_urdf;
    srdf::Model coman_srdf;
    std::string robot_name;
    iDynUtils iDynRobot; 
    std::map<walkman::drc::yarp_single_chain_interface*, kinematic_chain*> from_chains_to_kdl_chain;
    std::map<walkman::drc::yarp_single_chain_interface*,sensor_msgs::JointState> from_chain_to_joint_state_message;
    bool setJointNames(const std::string& name, sensor_msgs::JointState& _joint_state_msg);
    bool setEncodersPosition(walkman::drc::yarp_single_chain_interface* kc, sensor_msgs::JointState& _joint_state_msg);
    bool setEncodersSpeed(walkman::drc::yarp_single_chain_interface *kinematic_chain,
                        sensor_msgs::JointState &_joint_state_msg);
    bool setTorques(walkman::drc::yarp_single_chain_interface *kinematic_chain,
                    sensor_msgs::JointState &_joint_state_msg);
    void checkSRDF();
    void initialize_chain(std::string chain_name, kinematic_chain* kinem_chain);
};

#endif
