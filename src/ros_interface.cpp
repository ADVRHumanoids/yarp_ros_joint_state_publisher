#include "ros_interface.h"
#include <yarp/sig/all.h>

#define Deg2Rad(X) (X * M_PI/180.0)

static const char *kinematic_chains[5]  = {"torso", "right_leg", "left_leg", "right_arm", "left_arm"};
static const char *torso_joint_names[3] = {"WaistYaw", "WaistLat", "WaistSag"};
static const char *r_leg_joint_names[6] = {"RHipSag", "RHipLat", "RHipYaw", "RKneeSag", "RAnkLat", "RAnkSag"};
static const char *l_leg_joint_names[6] = {"LHipSag", "LHipLat", "LHipYaw", "LKneeSag", "LAnkLat", "LAnkSag"};
static const char *r_arm_joint_names[7] = {"RShSag", "RShLat", "RShYaw", "RElbj", "RForearmPlate", "RWrj1", "RWrj2"};
static const char *l_arm_joint_names[7] = {"LShSag", "LShLat", "LShYaw", "LElbj", "LForearmPlate", "LWrj1", "LWrj2"};

ros_interface::ros_interface():
    _kinematic_chains(),
    _n(),
    _joint_state_pub()
{
    _joint_state_pub = _n.advertise<sensor_msgs::JointState>("joint_states", 1);
}

void ros_interface::addKinematicChain(const boost::shared_ptr<yarp_kinematic_chain> &kinematic_chain)
{
    std::string name = kinematic_chain->getName();
    for(unsigned int i = 0; i < 5; ++i)
    {
        if(name.compare(kinematic_chains[i]) == 0)
        {
            _kinematic_chains.push_back(kinematic_chain);
            unsigned int i = _kinematic_chains.size();
            ROS_INFO("Added Kinematic Chain %s", name.c_str());
            ROS_INFO("# Kinematic Chains %i", i);
            return;
        }
    }
    ROS_ERROR("Kinematic chain %s unknown", name.c_str());
}

bool ros_interface::setJointNames(const std::string &name,
                                  sensor_msgs::JointState& _joint_state_msg)
{
    if(name.compare(kinematic_chains[0]) == 0) //torso
    {
        for(unsigned int i = 0; i < 3; ++i)
            _joint_state_msg.name.push_back(torso_joint_names[i]);
        return true;
    }
    if(name.compare(kinematic_chains[1]) == 0) //right_leg
    {
        for(unsigned int i = 0; i < 6; ++i)
            _joint_state_msg.name.push_back(r_leg_joint_names[i]);
        return true;
    }
    if(name.compare(kinematic_chains[2]) == 0) //left_leg
    {
        for(unsigned int i = 0; i < 6; ++i)
            _joint_state_msg.name.push_back(l_leg_joint_names[i]);
        return true;
    }
    if(name.compare(kinematic_chains[3]) == 0) //right_arm
    {
        for(unsigned int i = 0; i < 7; ++i)
            _joint_state_msg.name.push_back(r_arm_joint_names[i]);
        return true;
    }
    if(name.compare(kinematic_chains[4]) == 0) //left_arm
    {
        for(unsigned int i = 0; i < 7; ++i)
            _joint_state_msg.name.push_back(l_arm_joint_names[i]);
        return true;
    }
    return false;
}

bool ros_interface::setEncodersPosition(yarp_kinematic_chain &kc,
                                        sensor_msgs::JointState &_joint_state_msg)
{
    unsigned int nj = kc.getNumberOfJoints();
    yarp::sig::Vector temp(nj, 0.0);
    kc.getEncoders(temp);
    for(unsigned int i = 0; i < nj; ++i)
    {
        _joint_state_msg.position.push_back(Deg2Rad(temp[i]));
    }
    return true;
}

bool ros_interface::setEncodersSpeed(yarp_kinematic_chain &kc,
                                        sensor_msgs::JointState &_joint_state_msg)
{
    unsigned int nj = kc.getNumberOfJoints();
    yarp::sig::Vector temp(nj, 0.0);
    kc.getEncodersSpeed(temp);
    for(unsigned int i = 0; i < nj; ++i)
    {
        _joint_state_msg.velocity.push_back(Deg2Rad(temp[i]));
    }
    return true;
}

bool ros_interface::setTorques(yarp_kinematic_chain &kc,
                               sensor_msgs::JointState &_joint_state_msg)
{
    unsigned int nj = kc.getNumberOfJoints();
    yarp::sig::Vector temp(nj, 0.0);
    kc.getTorques(temp);
    for(unsigned int i = 0; i < nj; ++i)
    {
        _joint_state_msg.effort.push_back(temp[i]);
    }
    return true;
}

void ros_interface::publish()
{
    sensor_msgs::JointState joint_state_msg;
    for(unsigned int i = 0; i < _kinematic_chains.size(); ++i)
    {
        setJointNames(_kinematic_chains[i]->getName(), joint_state_msg);
        setEncodersPosition(*_kinematic_chains[i], joint_state_msg);
        setEncodersSpeed(*_kinematic_chains[i], joint_state_msg);
        setTorques(*_kinematic_chains[i], joint_state_msg);
    }
    joint_state_msg.header.stamp = ros::Time::now();
    _joint_state_pub.publish(joint_state_msg);
}
