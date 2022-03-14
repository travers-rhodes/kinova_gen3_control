#include <kinova_gen3_control/fake_kinova_network_connection.h>

FakeKinovaNetworkConnection::FakeKinovaNetworkConnection() 
{
  ROS_DEBUG("FakeNetwork: Creating connection");
  std::cout << "FakeNetwork: Creating connection" << std::endl;
}

FakeKinovaNetworkConnection::~FakeKinovaNetworkConnection() 
{
  ROS_DEBUG("FakeNetwork: Destroying connection");
  std::cout << "FakeNetwork: Destroying connection" << std::endl;
}

void FakeKinovaNetworkConnection::BaseSetServoingMode(const Kinova::Api::Base::ServoingModeInformation& servoing_mode)
{
  ROS_DEBUG("FakeNetwork: Setting Servoing Mode");
}

void FakeKinovaNetworkConnection::ActuatorSetControlMode(const Kinova::Api::ActuatorConfig::ControlModeInformation& servoing_mode, int actuator_device_id)
{
  ROS_DEBUG("FakeNetwork: Setting Control Mode");
}

void FakeKinovaNetworkConnection::CyclicRefresh(const Kinova::Api::BaseCyclic::Command& base_comand)
{
  ROS_DEBUG_THROTTLE(1, "FakeNetwork (throttled): Writing Joints");
}

Kinova::Api::BaseCyclic::Feedback FakeKinovaNetworkConnection::CyclicRefreshFeedback()
{
  ROS_DEBUG_THROTTLE(1, "FakeNetwork (throttled): Reading Joints");
  Kinova::Api::BaseCyclic::Feedback feedback;
  for (int i = 0; i < 7; i++)
  {
    Kinova::Api::BaseCyclic::ActuatorFeedback* actuators = feedback.add_actuators();
  }
  return feedback;
}
