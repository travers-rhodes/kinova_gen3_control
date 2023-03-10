#ifndef GAZEBO_KINOVA_NETWORK_CONNECTION_H 
#define GAZEBO_KINOVA_NETWORK_CONNECTION_H 

#include <ActuatorConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>
#include <kinova_gen3_control/kinova_network_connection.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

// When not testing individual joints, this should be 7
#define NUMBER_OF_JOINTS 7

// This object, when constructed, passes through all commands to gazebo joint effort interfaces
// This allows us to test much of our control logic with Gazebo 
class GazeboKinovaNetworkConnection : public KinovaNetworkConnection
{
  public:
    GazeboKinovaNetworkConnection(ros::NodeHandle &nh);
    ~GazeboKinovaNetworkConnection();
    // you must construct these methods in your implementation
    virtual void BaseSetServoingMode(const Kinova::Api::Base::ServoingModeInformation& servoing_mode);
    virtual void ActuatorSetControlMode(const Kinova::Api::ActuatorConfig::ControlModeInformation& servoing_mode, int actuator_device_id);
    virtual void CyclicRefresh(const Kinova::Api::BaseCyclic::Command& base_comand);
    virtual Kinova::Api::BaseCyclic::Feedback CyclicRefreshFeedback();
    void joint_state_callback(const sensor_msgs::JointState &message);

  private:
    ros::Subscriber sub_;
    std::vector<ros::Publisher> pub_;
    double cmd_[NUMBER_OF_JOINTS];
    double pos_[NUMBER_OF_JOINTS];
    double vel_[NUMBER_OF_JOINTS];
    double eff_[NUMBER_OF_JOINTS];
};

#endif // GAZEBO_KINOVA_NETWORK_CONNECTION_H 
