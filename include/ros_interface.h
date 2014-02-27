#ifndef _ROS_INTERFACE_H_
#define _ROS_INTERFACE_H_

#include "sensor_msgs/JointState.h"
#include "yarp_interface.h"
#include <boost/shared_ptr.hpp>
#include <ros/publisher.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

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
    urdf::Model coman_urdf;
    srdf::Model coman_srdf;
    std::vector< std::pair<std::string, int> > kinematic_chains;
    std::vector< std::pair<std::string, std::string> > chain_joint_maps;

    bool setJointNames(const std::string& name, sensor_msgs::JointState& _joint_state_msg);
    bool setEncodersPosition(yarp_kinematic_chain &kinematic_chain,
                             sensor_msgs::JointState& _joint_state_msg);
    bool setEncodersSpeed(yarp_kinematic_chain &kc,
                        sensor_msgs::JointState &_joint_state_msg);
    bool setTorques(yarp_kinematic_chain &kc,
                    sensor_msgs::JointState &_joint_state_msg);
    void checkSRDF()
    {
        std::vector<srdf::Model::Group> coman_groups = coman_srdf.getGroups();
        for(unsigned int i = 0; i < coman_groups.size(); ++i)
        {
            srdf::Model::Group group = coman_groups[i];
            std::pair< std::string, int> chain;
            chain.first = group.name_;
            chain.second = group.joints_.size();
            kinematic_chains.push_back(chain);
            ROS_INFO("GROUP %i name is: %s", i, group.name_.c_str());
            for(unsigned int j = 0; j < group.joints_.size(); ++j)
            {
                std::pair< std::string, std::string> chain_joint_map;
                chain_joint_map.first = group.name_;
                chain_joint_map.second = group.joints_[j];
                chain_joint_maps.push_back(chain_joint_map);
                std::string joint_name = group.joints_[j];
                ROS_INFO("  joint %i: %s", j, joint_name.c_str());
            }
        }
    }
};

#endif
