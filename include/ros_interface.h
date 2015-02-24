#ifndef _ROS_INTERFACE_H_
#define _ROS_INTERFACE_H_

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <idynutils/idynutils.h>
#include <boost/shared_ptr.hpp>
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <srdfdom/model.h>
#include <urdf/model.h>
#include <idynutils/yarp_single_chain_interface.h>
#include <idynutils/yarp_ft_interface.h>

struct chain_info_helper
{
    walkman::yarp_single_chain_interface* yarp_chain;
//     kinematic_chain* kin_chain;
    int index;
    yarp::sig::Vector temp_vector;
};

struct ft_info_helper
{
    std::shared_ptr<yarp_ft_interface> ftSensor;
    ros::Publisher ftPublisher;
    geometry_msgs::WrenchStamped ft_msg;
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
    std::string robot_name;
    iDynUtils iDynRobot;
    sensor_msgs::JointState message;
    std::vector<ft_info_helper> _ftSensors;
    bool setEncodersPosition(chain_info_helper& chain, sensor_msgs::JointState& _joint_state_msg);
    bool setEncodersSpeed(chain_info_helper& chain,sensor_msgs::JointState &_joint_state_msg);
    bool setTorques(chain_info_helper& chain,sensor_msgs::JointState &_joint_state_msg);
    bool initialize_chain(std::string chain_name, kinematic_chain *kinem_chain=0);

    bool loadForceTorqueSensors(iDynUtils &idynutils, const std::string& _moduleName);
    bool setFTMeasures(const ros::Time &t);

};

#endif
