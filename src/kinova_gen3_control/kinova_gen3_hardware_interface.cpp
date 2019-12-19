#include "kinova_gen3_control/kinova_gen3_hardware_interface.h"

void
InitializeLowLevelControl(
  Kinova::Api::Base::BaseClient *kinova_client,
  Kinova::Api::ActuatorConfig::ActuatorConfigClient *kinova_actuator_config_client) 
{
  std::cout << "Initialize low-level control" << std::endl;

  auto servoing_mode = Kinova::Api::Base::ServoingModeInformation();
  servoing_mode.set_servoing_mode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING);
  auto control_mode_message = Kinova::Api::ActuatorConfig::ControlModeInformation();
  control_mode_message.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::TORQUE);
  try
  {
    std::cout << "Set the servoing mode" << std::endl;
    // Set the base in low-level servoing mode
    kinova_client->SetServoingMode(servoing_mode);
    // Set last actuator in torque mode now that the command is equal to measure
    for (int i = 0; i < NUMBER_OF_JOINTS; i++)
    {
      // TEMP JUST WRIST: for now, just do wrist!
      int actuator_device_id = 7;
      // END TEMP JUST WRIST
      std::cout << "Set the actuator" << std::endl;
      kinova_actuator_config_client->SetControlMode(control_mode_message, actuator_device_id);
    }
  }
  catch (Kinova::Api::KDetailedException& ex)
  {
    std::cout << "API error: " << ex.what() << std::endl;
    throw;
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "Error: " << ex2.what() << std::endl;
    throw;
  }
  std::cout << "Low-level control initialized" << std::endl;
}

void
EndLowLevelControl(
  Kinova::Api::Base::BaseClient *kinova_client,
  Kinova::Api::ActuatorConfig::ActuatorConfigClient *kinova_actuator_config_client) 
{
  std::cout << "End low-level control";
  auto servoing_mode = Kinova::Api::Base::ServoingModeInformation();
  servoing_mode.set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  auto control_mode_message = Kinova::Api::ActuatorConfig::ControlModeInformation();
  control_mode_message.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::POSITION);
  try
  {
    // Set the base back to regular servoing mode
    kinova_client->SetServoingMode(servoing_mode);
    // Set last actuator to position mode 
    for (int i = 0; i < NUMBER_OF_JOINTS; i++)
    {
      // TEMP JUST WRIST: for now, just do wrist!
      int actuator_device_id = 7;
      // END TEMP JUST WRIST
      kinova_actuator_config_client->SetControlMode(control_mode_message, actuator_device_id);
    }
  }
  catch (Kinova::Api::KDetailedException& ex)
  {
    std::cout << "API error: " << ex.what() << std::endl;
    throw;
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "Error: " << ex2.what() << std::endl;
    throw;
  }
  std::cout << "Low-level control ended";
}

KinovaGen3HardwareInterface::KinovaGen3HardwareInterface(
  Kinova::Api::BaseCyclic::BaseCyclicClient *kinova_base_cyclic_client, 
  Kinova::Api::Base::BaseClient *kinova_base_client,
  Kinova::Api::ActuatorConfig::ActuatorConfigClient *kinova_actuator_config_client
  )
{

  // the client to talk to the kinova base at 1KHz
  kinova_cyclic_client_ = kinova_base_cyclic_client;
  // the client to talk to the base (slowly)
  kinova_client_ = kinova_base_client;
  // the client to talk to the base and configure actuators 
  kinova_actuator_config_client_ = kinova_actuator_config_client;

  InitializeLowLevelControl(kinova_client_, kinova_actuator_config_client_);

  std::cout << "Register hardware interface" << std::endl;
  std::vector<std::string> joint_names = {
    "joint_7",
  };

  // connect and register the joint state interfaces
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    hardware_interface::JointStateHandle state_handle(joint_names[i], &pos_[i], &vel_[i], &eff_[i]);
    jnt_state_interface_.registerHandle(state_handle);
  }

  registerInterface(&jnt_state_interface_);

  // connect and register the joint position interfaces
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    hardware_interface::JointHandle eff_handle(jnt_state_interface_.getHandle(joint_names[i]), &cmd_[i]);
    jnt_eff_interface_.registerHandle(eff_handle);
  }

  registerInterface(&jnt_eff_interface_);
  std::cout << "Hardware interfaces registered" << std::endl;
}

KinovaGen3HardwareInterface::~KinovaGen3HardwareInterface()
{
  EndLowLevelControl(kinova_client_, kinova_actuator_config_client_);
}

void KinovaGen3HardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  Kinova::Api::BaseCyclic::Command  base_command;

  // TEMP JUST WRIST: for now, just do wrist!
  for (int i = 0; i < 7; i++)
  {
    // Save the current actuator position, to avoid a following error
    base_command.add_actuators()->set_position(base_feedback_.actuators(i).position());
  }
  int relevant_joint = 6;
  // END TEMP JUST WRIST
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    // TEMP JUST WRIST: relevant_joint will need to change to i
    base_command.mutable_actuators(relevant_joint)->set_torque_joint(eff_[i]);
  }

  try
  {
    kinova_cyclic_client_->Refresh(base_command);
  } 
  catch (Kinova::Api::KDetailedException& ex)
  {
    std::cout << "API error: " << ex.what() << std::endl;
    throw; 
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "Error: " << ex2.what() << std::endl;
    throw; 
  }
}

void KinovaGen3HardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  base_feedback_ = kinova_cyclic_client_->RefreshFeedback();
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    // TEMP JUST WRIST: for now, just do wrist!
    i = 6;
    // END TEMP JUST WRIST
    pos_[0] = base_feedback_.actuators(i).position();
    vel_[0] = base_feedback_.actuators(i).velocity();
    eff_[0] = base_feedback_.actuators(i).torque();
  }
}
