#ifndef KINOVA_GEN3_HARDWARE_INTERFACE_H 
#define KINOVA_GEN3_HARDWARE_INTERFACE_H 

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>

#include <joint_limits_interface/joint_limits_interface.h>

#include <kinova_gen3_control/kinova_network_connection.h>


// 1-indexed first and last joint indices
// When not testing individual joints, this should be 1
#define FIRST_JOINT_INDEX 1
// When not testing individual joints, this should be 7
#define NUMBER_OF_JOINTS 7
// Set the max force of the gripper (between 0 and 100)
#define GRIPPER_MAX_FORCE 1.0

class KinovaGen3HardwareInterface : public hardware_interface::RobotHW
{
  public:
    KinovaGen3HardwareInterface(
      std::vector<std::string> joint_names,
      std::vector<joint_limits_interface::JointLimits> limits,
      std::shared_ptr<KinovaNetworkConnection> network_connection);
    ~KinovaGen3HardwareInterface();
    void write(const ros::Duration& period);
    void read();

  private:
    std::shared_ptr<KinovaNetworkConnection> network_connection_;
    std::vector<std::string> joint_names_;
    std::vector<joint_limits_interface::JointLimits> limits_;
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::EffortJointInterface jnt_eff_interface_;
    hardware_interface::VelocityJointInterface jnt_vel_interface_;
    joint_limits_interface::EffortJointSaturationInterface jnt_eff_limit_interface_;
    // no need to re-create this object every time it's used
    Kinova::Api::BaseCyclic::Feedback base_feedback_;
    double cmd_[NUMBER_OF_JOINTS];
    double pos_[NUMBER_OF_JOINTS];
    double vel_[NUMBER_OF_JOINTS];
    double eff_[NUMBER_OF_JOINTS];

   /* 
    * ABSTRACT: (copied from Kinova demo)
    * =========
    * On GEN3, gripper cyclic commands have 3 parameters used to control gripper
    * motion : position, velocity and force.
    *
    * These 3 parameters are always sent together and cannot be used independently.
    * Each parameter has absolute percentage values and their range are from 0% to
    * 100%.
    *
    * For gripper position, 0% is fully opened and 100% is fully closed.
    * For gripper speed, 0% is fully stopped and 100% is opening/closing (depending on
    * position used) at maximum speed.
    * Force parameter is used as a force limit to apply when closing or opening
    * the gripper. If this force limit is exceeded the gripper motion will stop.
    * 0% is the lowest force limit and 100% the maximum. if you use 0¸ it won't restart
    * until you change the target
    */

    ///////////
    // gripper commands (separate for convenience/readability)
    double grip_cmd_vel_;
    // this is the max force that the gripper will apply (scaled from 0 to 100)
    double grip_cmd_max_force_;
    ///////////
    // these are the read values from the gripper
    double grip_pos_;
    double grip_vel_;
    // this is the current (mA) consumed by the motor
    double grip_current_;
    // this is the voltage of the motor
    double grip_voltage_;
    // this is the temperature of the motor
    double grip_temperature_;
};
#endif // KINOVA_GEN3_HARDWARE_INTERFACE_H 
