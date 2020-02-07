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

class KinovaGen3HardwareInterface : public hardware_interface::RobotHW
{
  public:
    KinovaGen3HardwareInterface(
      std::vector<std::string> joint_names,
      std::vector<joint_limits_interface::JointLimits> limits,
      KinovaNetworkConnection* network_connection);
    ~KinovaGen3HardwareInterface();
    void write(const ros::Duration& period);
    void read();
  private:
    KinovaNetworkConnection* network_connection_;

    std::vector<std::string> joint_names_;
    std::vector<joint_limits_interface::JointLimits> limits_;
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::EffortJointInterface jnt_eff_interface_;
    joint_limits_interface::EffortJointSaturationInterface jnt_eff_limit_interface_;
    // no need to re-create this object every time it's used
    Kinova::Api::BaseCyclic::Feedback base_feedback_;
    double cmd_[NUMBER_OF_JOINTS];
    double pos_[NUMBER_OF_JOINTS];
    double vel_[NUMBER_OF_JOINTS];
    double eff_[NUMBER_OF_JOINTS];
};
