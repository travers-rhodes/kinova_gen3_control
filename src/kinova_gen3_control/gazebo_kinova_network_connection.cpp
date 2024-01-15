#include <kinova_gen3_control/gazebo_kinova_network_connection.h>
#include "angles/angles.h"

void GazeboKinovaNetworkConnection::joint_state_callback(const sensor_msgs::JointState& message)
{
  // In Gazebo, the first joint is the finger joint
  int has_gripper = 0;
  if (message.name[0] != "joint_1") {
    has_gripper = 1;
  }
  // and we want to mimic the weird negative effort that kinova does, too (sigh)
  // and we want to mimic the fact that kinova does degrees too. (DOUBLE sigh)
  for (int i = 0; i < GAZEBO_NUMBER_OF_JOINTS; i++) {
    pos_[i] = angles::to_degrees(message.position[i+has_gripper]);
    vel_[i] = angles::to_degrees(message.velocity[i+has_gripper]);
    eff_[i] = -message.effort[i+has_gripper];
  }
}

GazeboKinovaNetworkConnection::GazeboKinovaNetworkConnection(ros::NodeHandle &nh) 
{
  ROS_INFO("GazeboNetwork: Creating connection");
  sub_ = nh.subscribe("/my_gen3/joint_states", 1, &GazeboKinovaNetworkConnection::joint_state_callback, this);
  for (int i = 0; i < GAZEBO_NUMBER_OF_JOINTS; i++) {
    std::string joint_topic = "/my_gen3/joint_" + std::to_string(i+1) + "_effort_controller/command";
    uint32_t queue_size = 1;
    bool latch = true;
    pub_.push_back(nh.advertise<std_msgs::Float64>(joint_topic, queue_size, latch));
  }
  // before joint_states publish, be sure to initialize eff_ to 0's
  for (int i = 0; i < GAZEBO_NUMBER_OF_JOINTS; i++) {
    eff_[i] = 0;
  }
  // wait until joint_states is publishing before allowing the rest of the code to continue
  ros::topic::waitForMessage<sensor_msgs::JointState>("/my_gen3/joint_states");
  ros::topic::waitForMessage<sensor_msgs::JointState>("/my_gen3/joint_states");
}

GazeboKinovaNetworkConnection::~GazeboKinovaNetworkConnection() 
{
  ROS_INFO("GazeboNetwork: Destroying connection");
}

void GazeboKinovaNetworkConnection::BaseSetServoingMode(const Kinova::Api::Base::ServoingModeInformation& servoing_mode)
{
  ROS_INFO("GazeboNetwork: Setting Servoing Mode");
}

void GazeboKinovaNetworkConnection::ActuatorSetControlMode(const Kinova::Api::ActuatorConfig::ControlModeInformation& servoing_mode, int actuator_device_id)
{
  ROS_INFO("GazeboNetwork: Setting Control Mode");
}

void GazeboKinovaNetworkConnection::CyclicRefresh(const Kinova::Api::BaseCyclic::Command& base_command)
{
  ROS_DEBUG_THROTTLE(1, "GazeboNetwork (throttled): Writing Effort Commands");
  for (int i = 0; i < GAZEBO_NUMBER_OF_JOINTS; i++) {
    std_msgs::Float64 eff_cmd;
    eff_cmd.data = base_command.actuators()[i].torque_joint();
    pub_[i].publish(eff_cmd);
  }
}

Kinova::Api::BaseCyclic::Feedback GazeboKinovaNetworkConnection::CyclicRefreshFeedback()
{
  ROS_DEBUG_THROTTLE(1,"GazeboNetwork (throttled): Reading Joints");
  Kinova::Api::BaseCyclic::Feedback feedback;
  for (int i = 0; i < 7; i++)
  {
    Kinova::Api::BaseCyclic::ActuatorFeedback* actuator = feedback.add_actuators();
  }
  for (int i = 0; i < 7; i++)
  {
    feedback.mutable_actuators(i)->set_position(pos_[i]);
    feedback.mutable_actuators(i)->set_velocity(vel_[i]);
    feedback.mutable_actuators(i)->set_torque(eff_[i]);
  }
  ROS_DEBUG_THROTTLE(1,"GazeboNetwork (throttled): Done reading Joints");
  return feedback;
}
