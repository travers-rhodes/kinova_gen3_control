#ifndef KINOVA_GEN3_HARDWARE_INTERFACE_H 
#define KINOVA_GEN3_HARDWARE_INTERFACE_H 

#include <ros/ros.h>
#include "realtime_tools/realtime_publisher.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>

#include <joint_limits_interface/joint_limits_interface.h>

#include <kinova_gen3_control/kinova_network_connection.h>
#include <kinova_gen3_control/EffortCommand.h>


// 1-indexed first and last joint indices
// When not testing individual joints, this should be 1
#define FIRST_JOINT_INDEX 7
// When not testing individual joints, this should be 7
#define NUMBER_OF_JOINTS 1 

class KinovaGen3HardwareInterface : public hardware_interface::RobotHW
{
  public:
    KinovaGen3HardwareInterface(
      std::vector<std::string> joint_names,
      std::vector<joint_limits_interface::JointLimits> limits,
      std::shared_ptr<KinovaNetworkConnection> network_connection,
      ros::NodeHandle &nh);
    ~KinovaGen3HardwareInterface();
    void write(const ros::Duration& period);
    void read();

  private:
    void publish_throttled_debug_message(int severity, std::vector<float> before_limiting);
    std::shared_ptr<KinovaNetworkConnection> network_connection_;
    std::vector<std::string> joint_names_;
    std::vector<joint_limits_interface::JointLimits> limits_;
    // explicitly save pointers to these effort handles in a vector
    // since I think this will prevent some valgrind errors I was seeing in testing
    std::vector<hardware_interface::JointStateHandle> jnt_state_handles_;
    std::vector<hardware_interface::JointHandle> eff_handles_;
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::EffortJointInterface jnt_eff_interface_;
    joint_limits_interface::EffortJointSaturationInterface jnt_eff_limit_interface_;
    // no need to re-create this object every time it's used
    Kinova::Api::BaseCyclic::Feedback base_feedback_;
    double cmd_[NUMBER_OF_JOINTS];
    double pos_[NUMBER_OF_JOINTS];
    double vel_[NUMBER_OF_JOINTS];
    double eff_[NUMBER_OF_JOINTS];
    // For debugging, periodically log the actual commands we send to the kinova arm
    boost::shared_ptr<realtime_tools::RealtimePublisher<kinova_gen3_control::EffortCommand>> realtime_pub_;
    // send a debugging message every write 10 loops.
    // use this counter to count how many since last publication.
    // if last publish was NOT because of an error/warning
    // and the new message is because of an error/warning
    // then you can publish the new message regardless of counter
    int debug_msg_counter_ = 0;
    int last_publish_severity_ = 0;
};
#endif // KINOVA_GEN3_HARDWARE_INTERFACE_H 
