#ifndef MOCKED_POSITION_HARDWARE_INTERFACE_H 
#define MOCKED_POSITION_HARDWARE_INTERFACE_H 
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// http://wiki.ros.org/ros_control/Tutorials/Create%20your%20own%20hardware%20interface

class MockedPositionInterface : public hardware_interface::RobotHW
{
  public:
    MockedPositionInterface();
    void read();
    void write(const ros::Duration& period);

  private:
    std::vector<hardware_interface::JointStateHandle> joint_states_;
    std::vector<hardware_interface::JointHandle> pos_handles_;
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_pos_interface_;
    double cmd[7];
    double pos[7];
    double vel[7];
    double eff[7];
};
#endif // MOCKED_POSITION_HARDWARE_INTERFACE_H 
