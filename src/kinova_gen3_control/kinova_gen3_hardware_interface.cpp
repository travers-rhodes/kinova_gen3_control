#include "kinova_gen3_control/kinova_gen3_hardware_interface.h"
#include "angles/angles.h"

void
InitializeLowLevelControl(std::shared_ptr<KinovaNetworkConnection> network_connection)
{
  std::cout << "Initialize low-level control" << std::endl;

  auto servoing_mode = Kinova::Api::Base::ServoingModeInformation();
  servoing_mode.set_servoing_mode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING);
  auto control_mode_message = Kinova::Api::ActuatorConfig::ControlModeInformation();
  control_mode_message.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::CURRENT);
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

KinovaGen3HardwareInterface::KinovaGen3HardwareInterface(
  std::vector<std::string> joint_names,
  std::vector<joint_limits_interface::JointLimits> limits,
  std::shared_ptr<KinovaNetworkConnection> network_connection,
  ros::NodeHandle &nh) : joint_names_(joint_names), limits_(limits)
{
   realtime_pub_.reset(new realtime_tools::RealtimePublisher<kinova_gen3_control::EffortCommand>(nh, "low_level_effort_commands", 1));

  network_connection_ = network_connection;

  InitializeLowLevelControl(network_connection_);

  ROS_INFO("Register hardware interface");

  jnt_state_handles_.resize(NUMBER_OF_JOINTS);
  int array_index_offset = FIRST_JOINT_INDEX - 1;
  // connect and register the joint state interfaces
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    hardware_interface::JointStateHandle state_handle(joint_names_[i + array_index_offset], &pos_[i], &vel_[i], &eff_[i]);
    jnt_state_handles_[i] = state_handle;
    jnt_state_interface_.registerHandle(state_handle);
  }

  registerInterface(&jnt_state_interface_);

  eff_handles_.resize(NUMBER_OF_JOINTS);
  // connect and register the joint position interfaces
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    // note how a hardware_interface::JointHandle requires a hardware_interface::JointStateHandle in its constructor
    // in case you're wondering how this for loop relates to the for loop above
    hardware_interface::JointHandle eff_handle(jnt_state_interface_.getHandle(joint_names_[i + array_index_offset]), &cmd_[i]);
    eff_handles_[i] = eff_handle;
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
  EndLowLevelControl(network_connection_);
}

void KinovaGen3HardwareInterface::publish_throttled_debug_message(int severity, std::vector<float> before_limiting) {
  // add to throttle counter
  debug_msg_counter_ += 1;

  // publish if throttle counter high enough
  if (severity > last_publish_severity_ || debug_msg_counter_ >= 10) {
    if (realtime_pub_->trylock()){
      realtime_pub_->msg_.stamp = ros::Time::now();
      std::vector<float> command(std::begin(cmd_), std::end(cmd_));
      std::vector<float> read_position(std::begin(pos_), std::end(pos_));
      std::vector<float> read_velocity(std::begin(vel_), std::end(vel_));
      std::vector<float> read_effort(std::begin(eff_), std::end(eff_));
      std::vector<float> read_current(std::begin(curr_), std::end(curr_));
      realtime_pub_->msg_.command_before_limiting = before_limiting; 
      realtime_pub_->msg_.command = command; 
      realtime_pub_->msg_.read_position = read_position; 
      realtime_pub_->msg_.read_velocity = read_velocity; 
      realtime_pub_->msg_.read_effort = read_effort;
      realtime_pub_->msg_.read_current = read_current;
      // save last published severity and reset counter
      last_publish_severity_ = severity;
      debug_msg_counter_ = 0;
      realtime_pub_->unlockAndPublish();
    }
  }
}

void KinovaGen3HardwareInterface::write(const ros::Duration& period)
{
  int severity=0; // how important a debugging message of the current state would be

  std::vector<float> before_limiting(NUMBER_OF_JOINTS);
  for (int i=0; i<NUMBER_OF_JOINTS; i++){
    before_limiting[i] = cmd_[i];
  }

  Kinova::Api::BaseCyclic::Command  base_command;
  jnt_eff_limit_interface_.enforceLimits(period);

  bool joints_limited = false;
  bool joints_set_to_zero = false;
  for (int i=0; i<NUMBER_OF_JOINTS; i++){
    joints_limited = joints_limited || abs(before_limiting[i] - cmd_[i]) > 0.00001;
    joints_set_to_zero = joints_set_to_zero || abs(cmd_[i]) < 0.00001;
  }
  if (joints_limited){
    severity = 1;
    if (joints_set_to_zero) {
      severity = 2;
      std::stringstream errormsg;
      errormsg << "JointTorqueSetToZero: ";
      for (int i=0; i<NUMBER_OF_JOINTS; i++){
        if (abs(cmd_[i]) < 0.00001) {
          errormsg << "joint_" << i+1 << " with pos, vel, eff, desired of (" <<
            pos_[i] << ", " << vel_[i] << ", " << eff_[i] << ", " << before_limiting[i] << ")";
        }
      }
      // https://stackoverflow.com/questions/1374468/stringstream-string-and-char-conversion-confusion
      const std::string tmp = errormsg.str();
      // This error is expected because our joint limits
      // are stricter than the robots actual joint limits.
      // If the robot returns a position past our joint limits, we will
      // zero-out torque for that joint in that direction.
      //ROS_ERROR("%s", tmp.c_str());
    }
  }

  publish_throttled_debug_message(severity, before_limiting);

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
    //base_command.mutable_actuators(i + array_index_offset)->set_torque_joint(cmd_[i]);
    base_command.mutable_actuators(i + array_index_offset)->set_current_motor(cmd_[i]/curr_to_torque_factor_[i + array_index_offset]);

  }

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
    cmd_[i] = eff_[i]/2.0; // so that weird stuff doesn't happen before controller loads
                           // the factor of 2 reduction is to basically just set the control to close to 0
    // This read in curr_ is only used for debugging
    curr_[i] = base_feedback_.actuators(i + array_index_offset).current_motor(); // Amps
    if (isnan(cmd_[i])) {
      ROS_ERROR("Had a cmd of NAN!!!!!");
    }
  }
  
  if (NUMBER_OF_JOINTS == 7)
  {
    ROS_DEBUG_THROTTLE(1, "Read an effort of %f, %f, %f, %f, %f, %f, %f", cmd_[0], cmd_[1], cmd_[2], cmd_[3], cmd_[4], cmd_[5], cmd_[6]);
  }
  else
  {
    ROS_DEBUG_THROTTLE(1, "Read effort %03.4f, vel %03.4f, pos %03.4f", cmd_[0], vel_[0], pos_[0]);
  }
}
