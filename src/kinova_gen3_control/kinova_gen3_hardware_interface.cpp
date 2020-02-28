#include "kinova_gen3_control/kinova_gen3_hardware_interface.h"
#include "angles/angles.h"

void
InitializeLowLevelControl(std::shared_ptr<KinovaNetworkConnection> network_connection)
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
    network_connection->BaseSetServoingMode(servoing_mode);
    // Set last actuator in torque mode now that the command is equal to measure
    for (int i = 0; i < NUMBER_OF_JOINTS; i++)
    {
      // actuator_device_id is one-indexed
      int actuator_device_id_offset = FIRST_JOINT_INDEX;
      ROS_INFO("Set actuator %d control mode to low-level servoing", actuator_device_id_offset + i);
      network_connection->ActuatorSetControlMode(control_mode_message, actuator_device_id_offset + i);
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
EndLowLevelControl(std::shared_ptr<KinovaNetworkConnection> network_connection)
{
  ROS_INFO("End low-level control");
  std::cout << "End low-level control" << std::endl;
  auto servoing_mode = Kinova::Api::Base::ServoingModeInformation();
  servoing_mode.set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  auto control_mode_message = Kinova::Api::ActuatorConfig::ControlModeInformation();
  control_mode_message.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::POSITION);
  try
  {
    // Set the base back to regular servoing mode
    network_connection->BaseSetServoingMode(servoing_mode);
    // Set the actuators to position mode 
    for (int i = 0; i < NUMBER_OF_JOINTS; i++)
    {
      // actuator_device_id is one-indexed
      int actuator_device_id_offset = FIRST_JOINT_INDEX;
      ROS_INFO("Set actuator %d control mode to position control", actuator_device_id_offset + i);
      network_connection->ActuatorSetControlMode(control_mode_message, actuator_device_id_offset + i);
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

void
StopGripper(std::shared_ptr<KinovaNetworkConnection> network_connection, const Kinova::Api::BaseCyclic::Feedback &base_feedback)
{
  ROS_INFO("Stopping Gripper");
  std::cout << "Stopping Gripper" << std::endl;
  Kinova::Api::BaseCyclic::Command  base_command;

  // HARDCODE the total number of joints to 7 because we want to always
  // add position to each joint (even if only intentionally actuating fewer
  for (int i = 0; i < 7; i++)
  {
    // Save the current actuator position, to avoid a following error
    base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
  }
  int array_index_offset = FIRST_JOINT_INDEX - 1;
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    // Note that mutable_actuators is a 0-indexed array
    // note the negative one we need here because Kinova
    base_command.mutable_actuators(i + array_index_offset)->set_torque_joint(-base_feedback.actuators(i + array_index_offset).torque());
  }
  base_command.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
  // Fully opened
  base_command.mutable_interconnect()->mutable_gripper_command()->mutable_motor_cmd(0)->set_position(0.0);
  base_command.mutable_interconnect()->mutable_gripper_command()->mutable_motor_cmd(0)->set_velocity(0.0);
  base_command.mutable_interconnect()->mutable_gripper_command()->mutable_motor_cmd(0)->set_force(0.0);

  try
  {
    network_connection->CyclicRefresh(base_command);
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
  ROS_INFO("Stopped Gripper");
  std::cout << "Stopped Gripper" << std::endl;
}

KinovaGen3HardwareInterface::KinovaGen3HardwareInterface(
  std::vector<std::string> joint_names,
  std::vector<joint_limits_interface::JointLimits> limits,
  std::shared_ptr<KinovaNetworkConnection> network_connection) : joint_names_(joint_names), limits_(limits)
{
  std::string gripper_name = "robotiq_gripper";
  network_connection_ = network_connection;

  InitializeLowLevelControl(network_connection_);

  ROS_INFO("Register hardware interface");

  int array_index_offset = FIRST_JOINT_INDEX - 1;
  // connect and register the joint state interfaces
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    hardware_interface::JointStateHandle state_handle(joint_names_[i + array_index_offset], &pos_[i], &vel_[i], &eff_[i]);
    jnt_state_interface_.registerHandle(state_handle);
  }
    
  hardware_interface::JointStateHandle state_handle(gripper_name, &grip_pos_, &grip_vel_, &grip_current_);
  jnt_state_interface_.registerHandle(state_handle);

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
   
  // register gripper vel interface 
  hardware_interface::JointHandle pos_handle(jnt_state_interface_.getHandle(gripper_name), &grip_cmd_pos_);
  jnt_pos_interface_.registerHandle(pos_handle);
  registerInterface(&jnt_pos_interface_);

  ROS_INFO("Hardware interfaces registered");
}

KinovaGen3HardwareInterface::~KinovaGen3HardwareInterface()
{
  StopGripper(network_connection_, base_feedback_);
  EndLowLevelControl(network_connection_);
}

void KinovaGen3HardwareInterface::write(const ros::Duration& period)
{
  if (NUMBER_OF_JOINTS == 7)
  {
    ROS_DEBUG_THROTTLE(1, "Commanded effort of %f, %f, %f, %f, %f, %f, %f", cmd_[0], cmd_[1], cmd_[2], cmd_[3], cmd_[4], cmd_[5], cmd_[6]);
  }
  else
  {
   ROS_DEBUG_THROTTLE(1, "Commanded effort of %f", cmd_[0]);
  }

  Kinova::Api::BaseCyclic::Command  base_command;
  jnt_eff_limit_interface_.enforceLimits(period);

  if (NUMBER_OF_JOINTS == 7)
  {
    ROS_DEBUG_THROTTLE(1, "Writing an effort of %f, %f, %f, %f, %f, %f, %f", cmd_[0], cmd_[1], cmd_[2], cmd_[3], cmd_[4], cmd_[5], cmd_[6]);
  }
  else
  {
    ROS_DEBUG_THROTTLE(1, "Writing an effort of %f", cmd_[0]);
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

  //TODO Robotiq: Wrap this in an "ifHasGripper" before merging to master
  base_command.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
 
  // A little bit of smoothing never hurt anyone 
  double pos_error = grip_cmd_pos_ - grip_pos_;
  double grip_velocity_to_command = fabs(GRIPPER_VELOCITY_PROPORTION * pos_error);
  // velocity should never go above 100 or below 1 when you're sending a position command
  // the high-level robotiq controller will take care of stopping (note: velocity of 1.0/255 is min recorded velocity robotiq sees)
  grip_velocity_to_command = fmin(100.0, fmax(1.0/255, grip_velocity_to_command));
  ROS_DEBUG_THROTTLE(1, "Writing command to gripper %f, %f, %f", grip_cmd_pos_, grip_velocity_to_command, GRIPPER_MAX_FORCE);
  base_command.mutable_interconnect()->mutable_gripper_command()->mutable_motor_cmd(0)->set_position(grip_cmd_pos_);
  base_command.mutable_interconnect()->mutable_gripper_command()->mutable_motor_cmd(0)->set_velocity(grip_velocity_to_command);
  base_command.mutable_interconnect()->mutable_gripper_command()->mutable_motor_cmd(0)->set_force(GRIPPER_MAX_FORCE);

  try
  {
    network_connection_->CyclicRefresh(base_command);
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

void KinovaGen3HardwareInterface::read()
{
  base_feedback_ = network_connection_->CyclicRefreshFeedback();

  int array_index_offset = FIRST_JOINT_INDEX - 1;
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    // Note that actuators is a 0-indexed array
    pos_[i] = angles::normalize_angle(angles::from_degrees(base_feedback_.actuators(i + array_index_offset).position())); // originally degrees
    vel_[i] = angles::from_degrees(base_feedback_.actuators(i + array_index_offset).velocity()); // originally degrees per second
    // NOTA BENE: The torque returned by the feedback call is ``wrong''.
    // It is -1 times what you would expect.
    // That is, if the feedback returns a _positive_ torque, it means the motor
    // is pushing the arm toward a more _negative_ position.
    // (You can check this by putting the arm in a static position 
    // and seeing which way the joint must be pushing to counteract gravity)
    // We fix this oddity by multiplying the feedback by -1 here to give the
    // correct torque that the joint is currently applying.
    // An additional benefit of this fix is that you can now take the read effort
    // and write that to the robot as a command to have the robot maintain its current torque.
    // Note that this oddity is also reflected on the Kinova Dashboard 
    // (a positive torque there also means the joint is trying to make the position more negative)
    eff_[i] = -base_feedback_.actuators(i + array_index_offset).torque(); // originally Newton * meters
    cmd_[i] = eff_[i]; // so that weird stuff doesn't happen before controller loads
  }

  grip_pos_ = base_feedback_.interconnect().gripper_feedback().motor()[0].position();
  grip_vel_ = base_feedback_.interconnect().gripper_feedback().motor()[0].velocity();
  grip_current_ = base_feedback_.interconnect().gripper_feedback().motor()[0].current_motor();
  grip_voltage_ = base_feedback_.interconnect().gripper_feedback().motor()[0].voltage();
  grip_temperature_ = base_feedback_.interconnect().gripper_feedback().motor()[0].temperature_motor();
  grip_cmd_pos_ = grip_pos_; // default at startup: have gripper hold position
  ROS_DEBUG_THROTTLE(1, "Gripper Current: %03.4f, Voltage:  %03.4f, Temperature: %03.4f", grip_current_, grip_voltage_, grip_temperature_);
  
  if (NUMBER_OF_JOINTS == 7)
  {
    ROS_DEBUG_THROTTLE(1, "Read an effort of %f, %f, %f, %f, %f, %f, %f", cmd_[0], cmd_[1], cmd_[2], cmd_[3], cmd_[4], cmd_[5], cmd_[6]);
  }
  else
  {
    ROS_DEBUG_THROTTLE(1, "Read effort %03.4f, vel %03.4f, pos %03.4f", cmd_[0], vel_[0], pos_[0]);
  }
}
