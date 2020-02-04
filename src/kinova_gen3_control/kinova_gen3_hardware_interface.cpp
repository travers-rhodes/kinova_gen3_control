#include "kinova_gen3_control/kinova_gen3_hardware_interface.h"
#include "angles/angles.h"

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
    ROS_INFO("Set the servoing mode");
    // Set the base in low-level servoing mode
    kinova_client->SetServoingMode(servoing_mode);
    // Set last actuator in torque mode now that the command is equal to measure
    for (int i = 0; i < NUMBER_OF_JOINTS; i++)
    {
      // actuator_device_id is one-indexed
      int actuator_device_id_offset = FIRST_JOINT_INDEX;
      ROS_INFO("Set actuator %d control mode to low-level servoing", actuator_device_id_offset + i);
      kinova_actuator_config_client->SetControlMode(control_mode_message, actuator_device_id_offset + i);
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
  ROS_INFO("Low-level control initialized");
}

void
EndLowLevelControl(
  Kinova::Api::Base::BaseClient *kinova_client,
  Kinova::Api::ActuatorConfig::ActuatorConfigClient *kinova_actuator_config_client) 
{
  ROS_INFO("End low-level control");
  auto servoing_mode = Kinova::Api::Base::ServoingModeInformation();
  servoing_mode.set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  auto control_mode_message = Kinova::Api::ActuatorConfig::ControlModeInformation();
  control_mode_message.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::POSITION);
  try
  {
    // Set the base back to regular servoing mode
    kinova_client->SetServoingMode(servoing_mode);
    // Set the actuators to position mode 
    for (int i = 0; i < NUMBER_OF_JOINTS; i++)
    {
      // actuator_device_id is one-indexed
      int actuator_device_id_offset = FIRST_JOINT_INDEX;
      ROS_INFO("Set actuator %d control mode to position control", actuator_device_id_offset + i);
      kinova_actuator_config_client->SetControlMode(control_mode_message, actuator_device_id_offset + i);
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
  ROS_INFO("Low-level control ended");
}

KinovaGen3HardwareInterface::KinovaGen3HardwareInterface(
  std::vector<std::string> joint_names,
  std::vector<joint_limits_interface::JointLimits> limits,
  Kinova::Api::BaseCyclic::BaseCyclicClient *kinova_base_cyclic_client, 
  Kinova::Api::Base::BaseClient *kinova_base_client,
  Kinova::Api::ActuatorConfig::ActuatorConfigClient *kinova_actuator_config_client
  ) : joint_names_(joint_names), limits_(limits)
{

  // the client to talk to the kinova base at 1KHz
  kinova_cyclic_client_ = kinova_base_cyclic_client;
  // the client to talk to the base (slowly)
  kinova_client_ = kinova_base_client;
  // the client to talk to the base and configure actuators 
  kinova_actuator_config_client_ = kinova_actuator_config_client;

  InitializeLowLevelControl(kinova_client_, kinova_actuator_config_client_);

  ROS_INFO("Register hardware interface");

  int array_index_offset = FIRST_JOINT_INDEX - 1;
  // connect and register the joint state interfaces
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    hardware_interface::JointStateHandle state_handle(joint_names_[i + array_index_offset], &pos_[i], &vel_[i], &eff_[i]);
    jnt_state_interface_.registerHandle(state_handle);
  }

  registerInterface(&jnt_state_interface_);

  // connect and register the joint position interfaces
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    // note how a hardware_interface::JointHandle requires a hardware_interface::JointStateHandle in its constructor
    // in case you're wondering how this for loop relates to the for loop above
    hardware_interface::JointHandle eff_handle(jnt_state_interface_.getHandle(joint_names_[i + array_index_offset]), &cmd_[i]);
    jnt_eff_interface_.registerHandle(eff_handle);
  }


  // set up the joint limiting interface so we can use it in "write"
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    // Register handle in joint limits interface
    joint_limits_interface::EffortJointSaturationHandle eff_limit_handle(jnt_eff_interface_.getHandle(joint_names_[i + array_index_offset]), // We read the state and read/write the command
                                         limits_[i + array_index_offset]);       // Limits struct, copy constructor copies this
    jnt_eff_limit_interface_.registerHandle(eff_limit_handle);
  }

  registerInterface(&jnt_eff_interface_);
  ROS_INFO("Hardware interfaces registered");
}

KinovaGen3HardwareInterface::~KinovaGen3HardwareInterface()
{
  EndLowLevelControl(kinova_client_, kinova_actuator_config_client_);
}

void KinovaGen3HardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  if (NUMBER_OF_JOINTS == 7)
  {
    ROS_DEBUG_THROTTLE(0.05, "Commanded effort of %f, %f, %f, %f, %f, %f, %f", cmd_[0], cmd_[1], cmd_[2], cmd_[3], cmd_[4], cmd_[5], cmd_[6]);
  }
  else
  {
   // ROS_DEBUG_THROTTLE(0.05, "Commanded effort of %f", cmd_[0]);
  }

  Kinova::Api::BaseCyclic::Command  base_command;
  jnt_eff_limit_interface_.enforceLimits(period);

  if (NUMBER_OF_JOINTS == 7)
  {
    ROS_DEBUG_THROTTLE(0.05, "Writing an effort of %f, %f, %f, %f, %f, %f, %f", cmd_[0], cmd_[1], cmd_[2], cmd_[3], cmd_[4], cmd_[5], cmd_[6]);
  }
  else
  {
    ROS_DEBUG_THROTTLE(0.05, "Writing an effort of %f", cmd_[0]);
  }


  // HARDCODE the total number of joints to 7 because we want to always
  // add position to each joint (even if only intentionally actuating fewer
  for (int i = 0; i < 7; i++)
  {
    // Save the current actuator position, to avoid a following error
    base_command.add_actuators()->set_position(base_feedback_.actuators(i).position());
  }
  int array_index_offset = FIRST_JOINT_INDEX - 1;
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    // Note that mutable_actuators is a 0-indexed array
    base_command.mutable_actuators(i + array_index_offset)->set_torque_joint(cmd_[i]);
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

  int array_index_offset = FIRST_JOINT_INDEX - 1;
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    // Note that actuators is a 0-indexed array
    pos_[i] = angles::normalize_angle(angles::from_degrees(base_feedback_.actuators(i + array_index_offset).position())); // originally degrees
    vel_[i] = angles::from_degrees(base_feedback_.actuators(i + array_index_offset).velocity()); // originally degrees per second
    // NOTA BENE: FOR NO APPARENT REASON kinova multiplies the _READ_ torque by negative one
    // By doing this negative one, you can then send the effort right back as a torque command...
    // I AGREE that this makes no sense.
    eff_[i] = -base_feedback_.actuators(i + array_index_offset).torque(); // originally Newton * meters
    cmd_[i] = eff_[i]; // so that weird stuff doesn't happen before controller loads
  }
  
  if (NUMBER_OF_JOINTS == 7)
  {
    ROS_DEBUG_THROTTLE(0.05, "Read an effort of %f, %f, %f, %f, %f, %f, %f", cmd_[0], cmd_[1], cmd_[2], cmd_[3], cmd_[4], cmd_[5], cmd_[6]);
  }
  else
  {
    ROS_DEBUG_THROTTLE(0.05, "Read effort %03.4f, vel %03.4f, pos %03.4f", cmd_[0], vel_[0], pos_[0]);
  }
}
