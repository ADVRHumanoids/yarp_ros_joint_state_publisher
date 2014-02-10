#include "yarp_interface.h"
#include <yarp/sig/Vector.h>

#define toRad(X) (X*M_PI/180.0)

yarp_kinematic_chain::yarp_kinematic_chain(const std::string& name):
    _is_available(false),
    _name(name),
    _number_of_joints(0),
    _positions(1, 0.0),
    _speeds(1, 0.0),
    _torques(1, 0.0)
{
    std::string s = name;
    if(createPolyDriver(name, _polyDriver))
    {
        _polyDriver.view(_encodersMotor);
        _polyDriver.view(_torqueControl);
        _encodersMotor->getAxes(&_number_of_joints);
        _positions.resize(_number_of_joints, 0.0);
        _speeds.resize(_number_of_joints, 0.0);
        _torques.resize(_number_of_joints, 0.0);
        _is_available = true;

        printKinematicChainInfo();
    }
    else
    {
        s += " PolyDriver NOT Available";
        ROS_ERROR(s.c_str());
    }
}

yarp_kinematic_chain::~yarp_kinematic_chain()
{
    if(_is_available)
        _polyDriver.close();
}

bool yarp_kinematic_chain::createPolyDriver(const std::string& kinematic_chain, yarp::dev::PolyDriver& polyDriver)
{
    yarp::os::Property options;
    options.put("robot", "coman");
    options.put("device", "remote_controlboard");

    yarp::os::ConstString s;
    s = "/yarp_ros_joint_state_publisher/" + kinematic_chain;
    options.put("local", s.c_str());

    yarp::os::ConstString ss;
    ss = "/coman/" + kinematic_chain;
    options.put("remote", ss.c_str());

    polyDriver.open(options);
    if (!polyDriver.isValid())
        return false;
    return true;
}

void yarp_kinematic_chain::getEncoder(const int joint_id, double &position)
{
    if(_is_available && joint_id < _number_of_joints && joint_id >= 0)
        _encodersMotor->getEncoder(joint_id, &position);
    else
    {
        if(!_is_available)
        {
            ROS_ERROR("Kinematic Chain is not Available, I will write 0.0 as measure!");
            position = 0.0;
        }
        else
            ROS_ERROR("Joint id out of bounds or negative");
    }
}

double yarp_kinematic_chain::getEncoder(const int joint_id)
{
    double position = 0.0;
    getEncoder(joint_id, position);
    return position;
}

void yarp_kinematic_chain::getEncoders(yarp::sig::Vector& positions)
{
    if(_is_available && positions.size() == _number_of_joints)
        _encodersMotor->getEncoders(positions.data());
    else
    {
        if(!_is_available)
        {
            ROS_ERROR("Kinematic Chain is not Available, I will write all 0.0 as measure!");
            positions.resize(_number_of_joints, 0.0);
        }
        else
            ROS_ERROR("Position Vector WRONG SIZE!!! I will do nothing");
    }
}

yarp::sig::Vector& yarp_kinematic_chain::getEncoders()
{
    getEncoders(_positions);
    return _positions;
}

void yarp_kinematic_chain::getEncoderSpeed(const int joint_id, double &speed)
{
    if(_is_available && joint_id < _number_of_joints && joint_id >= 0)
        _encodersMotor->getEncoderSpeed(joint_id, &speed);
    else
    {
        if(!_is_available)
        {
            ROS_ERROR("Kinematic Chain is not Available, I will write 0.0 as measure!");
            speed = 0.0;
        }
        else
            ROS_ERROR("Joint id out of bounds or negative");
    }
}

double yarp_kinematic_chain::getEncoderSpeed(const int joint_id)
{
    double speed = 0.0;
    getEncoderSpeed(joint_id, speed);
    return speed;
}

void yarp_kinematic_chain::getEncodersSpeed(yarp::sig::Vector &speeds)
{
    if(_is_available && speeds.size() == _number_of_joints)
        _encodersMotor->getEncoderSpeeds(speeds.data());
    else
    {
        if(!_is_available)
        {
            ROS_ERROR("Kinematic Chain is not Available, I will write all 0.0 as measure!");
            speeds.resize(_number_of_joints, 0.0);
        }
        else
            ROS_ERROR("Position Vector WRONG SIZE!!! I will do nothing");
    }
}

yarp::sig::Vector& yarp_kinematic_chain::getEncodersSpeed()
{
    getEncoders(_speeds);
    return _speeds;
}

void yarp_kinematic_chain::getTorque(const int joint_id, double &torque)
{
    if(_is_available && joint_id < _number_of_joints && joint_id >= 0)
        _torqueControl->getTorque(joint_id, &torque);
    else
    {
        if(!_is_available)
        {
            ROS_ERROR("Kinematic Chain is not Available, I will write 0.0 as measure!");
            torque = 0.0;
        }
        else
            ROS_ERROR("Joint id out of bounds or negative");
    }
}

double yarp_kinematic_chain::getTorque(const int joint_id)
{
    double torque = 0.0;
    getTorque(joint_id, torque);
    return torque;
}

void yarp_kinematic_chain::getTorques(yarp::sig::Vector &torques)
{
    if(_is_available && torques.size() == _number_of_joints)
        _torqueControl->getTorques(torques.data());
    else
    {
        if(!_is_available)
        {
            ROS_ERROR("Kinematic Chain is not Available, I will write all 0.0 as measure!");
            torques.resize(_number_of_joints, 0.0);
        }
        else
            ROS_ERROR("Position Vector WRONG SIZE!!! I will do nothing");
    }
}

yarp::sig::Vector &yarp_kinematic_chain::getTorques()
{
    getTorques(_torques);
    return _torques;
}

void yarp_kinematic_chain::printKinematicChainInfo()
{
    ROS_INFO("Kinematic Chain name: %s", _name.c_str());
    ROS_INFO("# Joints: %i", _number_of_joints);
}


