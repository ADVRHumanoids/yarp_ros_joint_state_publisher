#include "ros_interface.h"
#include <idynutils//yarp_single_chain_interface.h>
#include <yarp/sig/all.h>
#include <ros/package.h>
#include <idynutils/idynutils.h>
#define Deg2Rad(X) (X * M_PI/180.0)

ros_interface::ros_interface(const std::string &robot_name_, const std::string &urdf_path, const std::string &srdf_path):
    _n(),
    _joint_state_pub(),robot_name(robot_name_),
    iDynRobot(robot_name_, urdf_path, srdf_path)
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
    loadForceTorqueSensors(iDynRobot, "yarp_ros_joint_state_publisher");
}


bool ros_interface::initialize_chain(std::string chain_name, kinematic_chain* kinem_chain)
{
    if (_initialized_status[chain_name])
        return true;
    chain_info_helper temp;
    temp.yarp_chain=new walkman::yarp_single_chain_interface(chain_name,"yarp_ros_joint_state_publisher",robot_name,false,walkman::controlTypes::none);
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

bool ros_interface::setEfforts(chain_info_helper &chain, sensor_msgs::JointState &_joint_state_msg)
{
    chain.yarp_chain->senseTorque(chain.temp_vector);
    for(unsigned int i = 0; i < chain.temp_vector.size(); ++i)
        _joint_state_msg.effort[chain.index+i]=chain.temp_vector[i]*chain.temp_vector[i];
    return true;
}

void ros_interface::publish()
{
    for(auto chain:_kinematic_chains)
    {
        setEncodersPosition(chain,message);
        setEncodersSpeed(chain,message);
        setEfforts(chain,message);
    }
    ros::Time t = ros::Time::now();
    message.header.stamp = t;
    setFTMeasures(t);

    _joint_state_pub.publish(message);

    for(unsigned int i = 0; i < _ftSensors.size(); ++i)
        _ftSensors.at(i).ftPublisher.publish(_ftSensors.at(i).ft_msg);
}

bool ros_interface::loadForceTorqueSensors(iDynUtils& idynutils, const std::string& _moduleName)
{
    std::vector<srdf::Model::Group> robot_groups = idynutils.robot_srdf->getGroups();
    for(auto group: robot_groups)
    {
        if (group.name_ == walkman::robot::force_torque_sensors)
        {
            if(group.joints_.size() > 0) {
                for(auto joint_name : group.joints_)
                {
                    std::cout << "ft sensors found on joint " << joint_name;

                    std::string reference_frame = idynutils.moveit_robot_model->getJointModel(joint_name)->
                            getChildLinkModel()->getName();

                    std::cout << " on frame " << reference_frame << ". Loading ft ..." << std::endl; std::cout.flush();

                    try {
                        std::shared_ptr<yarp_ft_interface> ft( new yarp_ft_interface(reference_frame,
                                                        _moduleName,
                                                        idynutils.getRobotName(), reference_frame) );

                        ft_info_helper tmpft;
                        tmpft.ftSensor = ft;
                        tmpft.ftPublisher = _n.advertise<geometry_msgs::WrenchStamped>(
                                    reference_frame + "/ft_sensor", 1);
                        tmpft.ft_msg.header.frame_id = reference_frame;

                        _ftSensors.push_back(tmpft);


                        std::cout << "ft on " << reference_frame << " loaded" << std::endl;
                    } catch(...) {
                        std::cerr << "Error loading " << reference_frame << " ft " << std::endl;
                        return false;}
                }
                return true;
            }
        }
    }
    std::cout << "Robot does not have any ft sensor" << std::endl;
    return false;
}

bool ros_interface::setFTMeasures(const ros::Time& t)
{
    for(unsigned int i = 0; i < _ftSensors.size(); ++i)
    {
        yarp::sig::Vector measures = _ftSensors.at(i).ftSensor->sense();

        if(measures.size() == 6)
        {
            _ftSensors.at(i).ft_msg.wrench.force.x = measures[0];
            _ftSensors.at(i).ft_msg.wrench.force.y = measures[1];
            _ftSensors.at(i).ft_msg.wrench.force.z = measures[2];
            _ftSensors.at(i).ft_msg.wrench.torque.x = measures[3];
            _ftSensors.at(i).ft_msg.wrench.torque.y = measures[4];
            _ftSensors.at(i).ft_msg.wrench.torque.z = measures[5];
        }
        else
        {
            ROS_WARN("Looks port for ft sensor in %s is not publishing values even if exists",
                     _ftSensors.at(i).ftSensor->getReferenceFrame().c_str());
            _ftSensors.at(i).ft_msg.wrench.force.x = 0.0;
            _ftSensors.at(i).ft_msg.wrench.force.y = 0.0;
            _ftSensors.at(i).ft_msg.wrench.force.z = 0.0;
            _ftSensors.at(i).ft_msg.wrench.torque.x = 0.0;
            _ftSensors.at(i).ft_msg.wrench.torque.y = 0.0;
            _ftSensors.at(i).ft_msg.wrench.torque.z = 0.0;
        }

        _ftSensors.at(i).ft_msg.header.stamp = t;
    }
}
