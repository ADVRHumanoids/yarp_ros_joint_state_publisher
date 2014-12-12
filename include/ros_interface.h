#ifndef _ROS_INTERFACE_H_
#define _ROS_INTERFACE_H_

#include <sensor_msgs/JointState.h>
//#include "yarp_interface.h"
#include <idynutils/idynutils.h>
#include <boost/shared_ptr.hpp>
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <srdfdom/model.h>
#include <urdf/model.h>
#include <idynutils/yarp_single_chain_interface.h>

struct chain_info_helper
{
    walkman::yarp_single_chain_interface* yarp_chain;
    kinematic_chain* kin_chain;
    int index;
    yarp::sig::Vector temp_vector;
};


class ros_interface
{
public:
    ros_interface(const std::string& robot_name_, const std::string& urdf_path,
                  const std::string& srdf_path);
    ~ros_interface(){}

    void publish();

private:
    //yarp data is here, the others map use pointers
    std::vector<chain_info_helper> _kinematic_chains;
    std::map<std::string, bool> _initialized_status;
    ros::NodeHandle _n;
    ros::Publisher _joint_state_pub;
    urdf::Model coman_urdf;
    srdf::Model coman_srdf;
    std::string robot_name;
    iDynUtils iDynRobot; 
    sensor_msgs::JointState message;
    bool setEncodersPosition(chain_info_helper& chain, sensor_msgs::JointState& _joint_state_msg);
    bool setEncodersSpeed(chain_info_helper& chain,sensor_msgs::JointState &_joint_state_msg);
    bool setTorques(chain_info_helper& chain,sensor_msgs::JointState &_joint_state_msg);
    bool initialize_chain(std::string chain_name, kinematic_chain *kinem_chain);
};

#endif
