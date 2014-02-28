#include "ros_interface.h"
#include <yarp/sig/all.h>
#include <ros/package.h>


#define Deg2Rad(X) (X * M_PI/180.0)

ros_interface::ros_interface():
    _kinematic_chains(),
    _n(),
    _joint_state_pub()
{
    std::string path_to_urdf = ros::package::getPath("coman_urdf") + "/urdf/coman.urdf";
    std::string path_to_srdf = ros::package::getPath("coman_srdf") + "/srdf/coman.srdf";

    if(coman_urdf.initFile(path_to_urdf))
    {
        ROS_INFO("Correctly loaded COMAN URDF!");
        if(coman_srdf.initFile(coman_urdf, path_to_srdf))
        {
            ROS_INFO("Correctly loaded COMAN SRDF!");
            checkSRDF();
        }
        else
            ROS_ERROR("Can not load COMAN SRDF!");
    }
    else
        ROS_ERROR("Can not load COMAN URDF!");

    _joint_state_pub = _n.advertise<sensor_msgs::JointState>("joint_states", 1);
}

void ros_interface::addKinematicChain(const boost::shared_ptr<yarp_kinematic_chain> &kinematic_chain)
{
    std::string name = kinematic_chain->getName();
    for(unsigned int i = 0; i < kinematic_chains.size(); ++i)
    {           
        if(name.compare(kinematic_chains[i].first) == 0)
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
    for(unsigned int i = 0; i < kinematic_chains.size(); ++i)
    {
        if(name.compare(kinematic_chains[i].first) == 0)
        {
            for(unsigned int j = 0; j < chain_joint_maps.size(); ++j)
            {
                if(kinematic_chains[i].first.compare(chain_joint_maps[j].first) == 0)
                {
                    _joint_state_msg.name.push_back(chain_joint_maps[j].second);
                    //ROS_INFO("%s joint belongs to %s chain", chain_joint_maps[j].second.c_str(), kinematic_chains[i].first.c_str());
                }
            }
            return true;
        }
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
