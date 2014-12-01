#include "ros_interface.h"
#include <idynutils//yarp_single_chain_interface.h>
#include <yarp/sig/all.h>
#include <ros/package.h>
#include <idynutils/idynutils.h>
#define Deg2Rad(X) (X * M_PI/180.0)

ros_interface::ros_interface():
    _n(),
    _joint_state_pub(),robot_name("coman"),
    iDynRobot(robot_name) //This can be changed or loaded on runtime!!
{
    _joint_state_pub = _n.advertise<sensor_msgs::JointState>("joint_states", 1);

    bool done=false;
    int counter=0;
    _initialized_status[walkman::robot::left_arm]=false;
    _initialized_status[walkman::robot::torso]=false;
    _initialized_status[walkman::robot::right_leg]=false;
    _initialized_status[walkman::robot::right_arm]=false;
    _initialized_status[walkman::robot::left_leg]=false;
    while (!done)
    {
        int initialized=0;

        if (initialize_chain(walkman::robot::left_arm,&iDynRobot.left_arm)) initialized++;

        if (initialize_chain(walkman::robot::left_leg,&iDynRobot.left_leg)) initialized++;

        if (initialize_chain(walkman::robot::right_arm,&iDynRobot.right_arm)) initialized++;

        if (initialize_chain(walkman::robot::right_leg,&iDynRobot.right_leg)) initialized++;

        if (initialize_chain(walkman::robot::torso,&iDynRobot.torso)) initialized++;

        if (initialized==0)
        {
            std::cout<<"could not initialize any chains, is the robot running?"<<std::endl;
            std::cout<<"Waiting for a robot"<<std::endl;
            sleep(1);
            continue;
        }
        if (initialized>0 && initialized<5)
        {
            std::cout<<"Warning: could not initialize some chains, does the robot have all the chains?"<<std::endl;
            if (counter==initialized)
            {
                std::cout<<"Some chains were not initialized, I will go on without them"<<std::endl;
                done=true;
            }
            else
            {
                std::cout<<"Some chains were not initialized, I will try again"<<std::endl;
                counter=initialized;
                sleep(2);
                done=false;
                continue;
            }
        }
        done=true;
    }
}


bool ros_interface::initialize_chain(std::string chain_name, kinematic_chain* kinem_chain)
{
    if (_initialized_status[chain_name])
        return true;
    chain_info_helper temp;
    temp.yarp_chain=new walkman::yarp_single_chain_interface(chain_name,"yarp_ros_joint_state_publisher",robot_name,false,WALKMAN_CM_NONE);
    if (!temp.yarp_chain->isAvailable)
    {
        std::cout<<"cannot initialize chain "<<chain_name<<std::endl;
        delete temp.yarp_chain;
        _initialized_status[chain_name]=false;
        return false;
    }
    _kinematic_chains.push_back(temp);
    _kinematic_chains.back().kin_chain=kinem_chain;
    _kinematic_chains.back().temp_vector.resize(kinem_chain->getNrOfDOFs());
    _kinematic_chains.back().temp_vector.zero();
    _kinematic_chains.back().index=message.position.size();
    message.effort.resize(message.effort.size()+kinem_chain->getNrOfDOFs());
    message.position.resize(message.position.size()+kinem_chain->getNrOfDOFs());
    message.velocity.resize(message.velocity.size()+kinem_chain->getNrOfDOFs());
    message.name.insert(message.name.end(),kinem_chain->joint_names.begin(),kinem_chain->joint_names.end());
    _initialized_status[chain_name]=true;
    return true;
}

bool ros_interface::setEncodersPosition(chain_info_helper &chain, sensor_msgs::JointState &_joint_state_msg)
{
    chain.yarp_chain->sensePosition(chain.temp_vector);
    for(unsigned int i = 0; i < chain.temp_vector.size(); ++i)
    {
        _joint_state_msg.position[chain.index+i]=Deg2Rad(chain.temp_vector[i]);
    }
    return true;
}

bool ros_interface::setEncodersSpeed(chain_info_helper &chain, sensor_msgs::JointState &_joint_state_msg)
{
    chain.yarp_chain->senseVelocity(chain.temp_vector);
    for(unsigned int i = 0; i < chain.temp_vector.size(); ++i)
    {
        _joint_state_msg.velocity[chain.index+i]=Deg2Rad(chain.temp_vector[i]);
    }
    return true;
}

bool ros_interface::setTorques(chain_info_helper &chain, sensor_msgs::JointState &_joint_state_msg)
{
    chain.yarp_chain->senseTorque(chain.temp_vector);
    for(unsigned int i = 0; i < chain.temp_vector.size(); ++i)
    {
        _joint_state_msg.effort[chain.index+i]=chain.temp_vector[i];
    }
    return true;
}

void ros_interface::publish()
{
    for(auto chain:_kinematic_chains)
    {
        setEncodersPosition(chain,message);
        setEncodersSpeed(chain,message);
        setTorques(chain,message);
    }
    message.header.stamp = ros::Time::now();
    _joint_state_pub.publish(message);
}
