#include "ros_interface.h"
#include "drc_shared/yarp_single_chain_interface.h"
#include <yarp/sig/all.h>
#include <ros/package.h>
#include <drc_shared/idynutils.h>
#define Deg2Rad(X) (X * M_PI/180.0)

ros_interface::ros_interface():
    _n(),
    _joint_state_pub(),robot_name("coman"),
    iDynRobot(robot_name) //This can be changed or loaded on runtime!!
{
    _joint_state_pub = _n.advertise<sensor_msgs::JointState>("joint_states", 1);

    initialize_chain(walkman::robot::left_arm,&iDynRobot.left_arm);

    initialize_chain(walkman::robot::left_leg,&iDynRobot.left_leg);

    initialize_chain(walkman::robot::right_arm,&iDynRobot.right_arm);

    initialize_chain(walkman::robot::right_leg,&iDynRobot.right_leg);

    initialize_chain(walkman::robot::torso,&iDynRobot.torso);

}

void ros_interface::initialize_chain(std::string chain_name, kinematic_chain* kinem_chain)
{
    sensor_msgs::JointState temp;
    _kinematic_chains.emplace_back(new walkman::drc::yarp_single_chain_interface(chain_name,"yarp_ros_joint_state_publisher",robot_name,false));
    from_chains_to_kdl_chain.emplace(_kinematic_chains.back(),kinem_chain);
    temp.effort.resize(kinem_chain->getNrOfDOFs());
    temp.position.resize(kinem_chain->getNrOfDOFs());
    temp.velocity.resize(kinem_chain->getNrOfDOFs());
    temp.name=kinem_chain->joint_names;
    from_chain_to_joint_state_message[_kinematic_chains.back()]=temp;
}

bool ros_interface::setEncodersPosition(walkman::drc::yarp_single_chain_interface* kc,
                                        sensor_msgs::JointState &_joint_state_msg)
{
    unsigned int nj = kc->getNumberOfJoints();
    yarp::sig::Vector temp(nj, 0.0);
    kc->sensePosition(temp);
    for(unsigned int i = 0; i < nj; ++i)
    {
        _joint_state_msg.position[i]=Deg2Rad(temp[i]);
    }
    return true;
}

bool ros_interface::setEncodersSpeed(walkman::drc::yarp_single_chain_interface* kc,
                                        sensor_msgs::JointState &_joint_state_msg)
{
    unsigned int nj = kc->getNumberOfJoints();
    yarp::sig::Vector temp(nj, 0.0);
    kc->senseVelocity(temp);
    for(unsigned int i = 0; i < nj; ++i)
    {
        _joint_state_msg.velocity[i]=Deg2Rad(temp[i]);
    }
    return true;
}

bool ros_interface::setTorques(walkman::drc::yarp_single_chain_interface* kc,
                               sensor_msgs::JointState &_joint_state_msg)
{
    unsigned int nj = kc->getNumberOfJoints();
    yarp::sig::Vector temp(nj, 0.0);
    kc->senseTorque(temp);
    for(unsigned int i = 0; i < nj; ++i)
    {
        _joint_state_msg.effort[i]=temp[i];
    }
    return true;
}

void ros_interface::publish()
{
    for(auto joint_state_msg : from_chain_to_joint_state_message)
    {
        setEncodersPosition(joint_state_msg.first, joint_state_msg.second);
        setEncodersSpeed(joint_state_msg.first, joint_state_msg.second);
        setTorques(joint_state_msg.first, joint_state_msg.second);
        joint_state_msg.second.header.stamp = ros::Time::now();
        _joint_state_pub.publish(joint_state_msg.second);
    }

}
