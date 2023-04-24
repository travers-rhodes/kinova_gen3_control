#include <mock_position_joint_hardware_interface/mocked_position_joint_hardware_interface.h>

// http://wiki.ros.org/ros_control/Tutorials/Create%20your%20own%20hardware%20interface
MockedPositionInterface::MockedPositionInterface() 
{ 
  std::vector<std::string> joint_names = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7"};
  for (int i = 0; i < 7; i++) {
    pos[i] = vel[i] = eff[i] = 0;
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
    joint_states_.push_back(state_handle);
    jnt_state_interface_.registerHandle(state_handle);
  }

  registerInterface(&jnt_state_interface_);

  for (int i = 0; i < 7; i++) {
    // connect and register the joint position interface
    cmd[i] = 0;
    hardware_interface::JointHandle pos_handle(jnt_state_interface_.getHandle(joint_names[i]), &cmd[i]);
    pos_handles_.push_back(pos_handle);
    jnt_pos_interface_.registerHandle(pos_handle);
  }

  registerInterface(&jnt_pos_interface_);
}

void MockedPositionInterface::read() {
  // do nothing
}

void MockedPositionInterface::write(const ros::Duration& period) {
  for (int i = 0; i < 7; i++) {
    pos[i] = cmd[i];
  }
}
