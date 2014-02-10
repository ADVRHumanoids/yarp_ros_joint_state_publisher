#ifndef _ROS_INTERFACE_H_
#define _ROS_INTERFACE_H_

#include "sensor_msgs/JointState.h"
#include "yarp_interface.h"
#include <boost/shared_ptr.hpp>
#include <ros/publisher.h>

class ros_interface
{
public:
    ros_interface();
    ~ros_interface(){}

    void addKinematicChain(const boost::shared_ptr<yarp_kinematic_chain>& kinematic_chain);
    void publish();

private:
    std::vector<boost::shared_ptr<yarp_kinematic_chain> > _kinematic_chains;
    ros::NodeHandle _n;
    ros::Publisher _joint_state_pub;

    bool setJointNames(const std::string& name, sensor_msgs::JointState& _joint_state_msg);
    bool setEncodersPosition(yarp_kinematic_chain &kinematic_chain,
                             sensor_msgs::JointState& _joint_state_msg);
    bool setEncodersSpeed(yarp_kinematic_chain &kc,
                        sensor_msgs::JointState &_joint_state_msg);
    bool setTorques(yarp_kinematic_chain &kc,
                    sensor_msgs::JointState &_joint_state_msg);
};

#endif
