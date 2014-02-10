#ifndef _YARP_INTERFACE_H_
#define _YARP_INTERFACE_H_

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <sensor_msgs/JointState.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <yarp/sig/all.h>
#include <boost/scoped_ptr.hpp>

class yarp_kinematic_chain
{
public:
    yarp_kinematic_chain(const std::string &name);
    ~yarp_kinematic_chain();

    bool isAvailable(){return _is_available;}
    int getNumberOfJoints(){return _number_of_joints;}
    void printKinematicChainInfo();
    std::string getName(){return _name;}

    /** All the following functions get Degrees **/
    void getEncoder(const int joint_id, double& position);
    double getEncoder(const int joint_id);
    void getEncoders(yarp::sig::Vector& positions);
    yarp::sig::Vector &getEncoders();

    /** All the following functions get Degrees/Sec **/
    void getEncoderSpeed(const int joint_id, double& speed);
    double getEncoderSpeed(const int joint_id);
    void getEncodersSpeed(yarp::sig::Vector& speeds);
    yarp::sig::Vector &getEncodersSpeed();

    /** All the following functions get Nm **/
    void getTorque(const int joint_id, double& torque);
    double getTorque(const int joint_id);
    void getTorques(yarp::sig::Vector& torques);
    yarp::sig::Vector &getTorques();


private:
    yarp::dev::PolyDriver _polyDriver;
    bool _is_available;
    std::string _name;
    int _number_of_joints;
    yarp::dev::IEncodersTimed *_encodersMotor;
    yarp::dev::ITorqueControl *_torqueControl;
    yarp::sig::Vector _positions; //[deg]
    yarp::sig::Vector _speeds; //[deg/sec]
    yarp::sig::Vector _torques; //[Nm]

    bool createPolyDriver(const std::string& kinematic_chain, yarp::dev::PolyDriver& polyDriver);
};
#endif
