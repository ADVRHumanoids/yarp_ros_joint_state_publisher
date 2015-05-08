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


    _initialized_status[walkman::robot::left_arm]=false;
    _initialized_status[walkman::robot::torso]=false;
    _initialized_status[walkman::robot::right_leg]=false;
    _initialized_status[walkman::robot::right_arm]=false;
    _initialized_status[walkman::robot::left_leg]=false;
    _initialized_status[walkman::robot::head]=false;

    if(!initialize_chain(walkman::robot::left_arm,&iDynRobot.left_arm))
        ROS_WARN("CAN NOT INITIALIZE EXPECTED CHAIN left_arm!");

    if(!initialize_chain(walkman::robot::left_leg,&iDynRobot.left_leg))
        ROS_WARN("CAN NOT INITIALIZE EXPECTED CHAIN left_leg!");

    if(!initialize_chain(walkman::robot::right_arm,&iDynRobot.right_arm))
        ROS_WARN("CAN NOT INITIALIZE EXPECTED CHAIN right_arm!");

    if(!initialize_chain(walkman::robot::right_leg,&iDynRobot.right_leg))
        ROS_WARN("CAN NOT INITIALIZE EXPECTED CHAIN right_leg!");

    if(!initialize_chain(walkman::robot::torso,&iDynRobot.torso))
        ROS_WARN("CAN NOT INITIALIZE EXPECTED CHAIN torso!");

    if(!initialize_chain(walkman::robot::head, &iDynRobot.head))
        ROS_WARN("CAN NOT INITIALIZE EXPECTED CHAIN head!");

    loadForceTorqueSensors(iDynRobot, "yarp_ros_joint_state_publisher");
    loadImuSensors(iDynRobot, "yarp_ros_joint_state_publisher");
}


bool ros_interface::initialize_chain(std::string chain_name, kinematic_chain* kinem_chain)
{
    if (_initialized_status[chain_name])
        return true;
    chain_info_helper temp;
    temp.yarp_chain=new walkman::yarp_single_chain_interface(chain_name,"yarp_ros_joint_state_publisher",robot_name,false,walkman::controlTypes::none);
    if (!temp.yarp_chain->isAvailable)
    {
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
        _joint_state_msg.position[chain.index+i]=Deg2Rad(chain.temp_vector[i]);
    return true;
}

bool ros_interface::setEncodersSpeed(chain_info_helper &chain, sensor_msgs::JointState &_joint_state_msg)
{
    chain.yarp_chain->senseVelocity(chain.temp_vector);
    for(unsigned int i = 0; i < chain.temp_vector.size(); ++i)
        _joint_state_msg.velocity[chain.index+i]=Deg2Rad(chain.temp_vector[i]);
    return true;
}

bool ros_interface::setEfforts(chain_info_helper &chain, sensor_msgs::JointState &_joint_state_msg)
{
    chain.yarp_chain->senseTorque(chain.temp_vector);
    for(unsigned int i = 0; i < chain.temp_vector.size(); ++i)
        _joint_state_msg.effort[chain.index+i] = chain.temp_vector[i];
    return true;
}

void ros_interface::publish()
{
    for(unsigned int i = 0; i < _kinematic_chains.size(); ++i)
    {
        if(_initialized_status[_kinematic_chains[i].kin_chain->chain_name]){
            setEncodersPosition(_kinematic_chains[i],message);
            setEncodersSpeed(_kinematic_chains[i],message);
            setEfforts(_kinematic_chains[i],message);
        }

    }
    ros::Time t = ros::Time::now();
    message.header.stamp = t;
    setFTMeasures(t);
    setIMUMeasures(t);

    _joint_state_pub.publish(message);

    for(unsigned int i = 0; i < _ftSensors.size(); ++i)
        _ftSensors.at(i).ftPublisher.publish(_ftSensors.at(i).ft_msg);
    for(unsigned int i = 0; i < _imuSensors.size(); ++i)
        _imuSensors.at(i).imuPublisher.publish(_imuSensors.at(i).imu_msg);
}

bool ros_interface::loadImuSensors(iDynUtils &idynutils, const std::string& _moduleName)
{
    std::vector<srdf::Model::Group> robot_groups = idynutils.robot_srdf->getGroups();
    for(unsigned int i = 0; i < robot_groups.size(); ++i)
    {
        srdf::Model::Group group = robot_groups[i];
        if(group.name_ == walkman::robot::imu_sensors){
            if(group.links_.size() > 0){
                for(unsigned int j = 0; j < group.links_.size(); ++j)
                {
                    std::string link = group.links_[j];
                    std::cout << "imu sensor found on link "<<link<<std::endl;

                    try{
                        std::shared_ptr<yarp_IMU_interface> imu( new yarp_IMU_interface(_moduleName,
                                                        idynutils.getRobotName(), true, link) );

                        imu_info_helper tmpimu;
                        tmpimu.imuSensor = imu;
                        tmpimu.imuPublisher = _n.advertise<sensor_msgs::Imu>(
                                    link + "/imu_sensor", 1);
                        tmpimu.imu_msg.header.frame_id = link;

                        for(unsigned int k = 0; k < tmpimu.imu_msg.angular_velocity_covariance.size(); ++k){
                            tmpimu.imu_msg.angular_velocity_covariance[k] = 0.0;
                            tmpimu.imu_msg.linear_acceleration_covariance[k] = 0.0;
                            tmpimu.imu_msg.orientation_covariance[k] = 0.0;}

                        _imuSensors.push_back(tmpimu);

                        std::cout << "imu on " << link << " loaded" << std::endl;
                    } catch(...) {
                        std::cerr << "Error loading " << link << " imu " << std::endl;
                        return false;}
                }
                return true;
            }
        }
    }
    std::cout << "Robot does not have any imu sensor" << std::endl;
    return false;
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
                    std::string reference_frame = idynutils.moveit_robot_model->getJointModel(joint_name)->
                            getChildLinkModel()->getName();

                    ROS_INFO("ft sensors found on joint %s on frame %s. Loading ft ...", joint_name.c_str(), reference_frame.c_str());

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

                        ROS_INFO("ft on %s loaded!", reference_frame.c_str());
                    } catch(...) {
                        ROS_ERROR("Error loading %s ft!", reference_frame.c_str());
                        return false;}
                }
                return true;
            }
        }
    }
    ROS_INFO("Robot does not have any ft sensor");
    return false;
}

bool ros_interface::setIMUMeasures(const ros::Time &t)
{
    for(unsigned int i = 0; i < _imuSensors.size(); ++i)
    {
        KDL::Rotation orientation;
        KDL::Vector linearAcc;
        KDL::Vector angularVel;
        _imuSensors.at(i).imuSensor->sense(orientation, linearAcc, angularVel);

        double x, y, z, w;
        orientation.GetQuaternion(x, y, z, w);

        _imuSensors.at(i).imu_msg.orientation.x = x;
        _imuSensors.at(i).imu_msg.orientation.y = y;
        _imuSensors.at(i).imu_msg.orientation.z = z;
        _imuSensors.at(i).imu_msg.orientation.w = w;
        _imuSensors.at(i).imu_msg.angular_velocity.x = angularVel.x();
        _imuSensors.at(i).imu_msg.angular_velocity.y = angularVel.y();
        _imuSensors.at(i).imu_msg.angular_velocity.z = angularVel.z();
        _imuSensors.at(i).imu_msg.linear_acceleration.x = linearAcc.x();
        _imuSensors.at(i).imu_msg.linear_acceleration.y = linearAcc.y();
        _imuSensors.at(i).imu_msg.linear_acceleration.z = linearAcc.z();

        _imuSensors.at(i).imu_msg.header.stamp = t;
    }
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
            ROS_WARN_ONCE("Looks port for ft sensor in %s is not publishing values even if exists",
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
