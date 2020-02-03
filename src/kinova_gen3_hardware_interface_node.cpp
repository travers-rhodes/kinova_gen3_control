#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>


#include "kinova_gen3_control/kinova_gen3_hardware_interface.h"
#include "kinova_gen3_control/kinova_gen3_network_connection.h"

//https://slaterobots.com/blog/5abd8a1ed4442a651de5cb5b/how-to-implement-ros_control-on-a-custom-robot
int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinova_gen3_hardware_interface");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::cout << "Parsing URDF" << std::endl;
  urdf::Model urdf_robot;
  // load urdf from "robot_description"
  // TODO don't hard-code this string
  urdf_robot.initParam("robot_description");

  std::cout << "Creating network connection" << std::endl;
  KinovaGen3NetworkConnection kinova_gen3_connection;

  std::cout << "Creating hardware interface" << std::endl;
  std::vector<std::string> joint_names = {
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7"
  };
  std::vector<joint_limits_interface::JointLimits> limits_list;
  for (int i = 0; i < joint_names.size(); i++)
  {
    joint_limits_interface::JointLimits limits;
    std::shared_ptr<const urdf::Joint> urdf_joint = urdf_robot.getJoint(joint_names[i]);
    const bool urdf_limits_ok = getJointLimits(urdf_joint, limits);
    // Populate joint limits from the ros parameter server
    // Limits specified in the parameter server overwrite existing values in 'limits' 
    // Limits not specified in the parameter server preserve their existing values
    const bool rosparam_limits_ok = getJointLimits(joint_names[i], nh, limits);
    limits_list.push_back(limits);
  }

  KinovaGen3HardwareInterface robot(
      joint_names,
      limits_list,
      kinova_gen3_connection.base_cyclic,
      kinova_gen3_connection.base,
      kinova_gen3_connection.actuator_config);

  std::cout << "Starting hardware manager" << std::endl;
  controller_manager::ControllerManager controller_manager(&robot, nh);

  int loop_hz = 1000;
  ros::Duration update_freq = ros::Duration(1.0/loop_hz);
  ros::Time previous = ros::Time::now();
  ros::Time current = ros::Time::now();

  while (ros::Time::now().toSec() == 0)
  {
    std::cout << "waiting for clock to publish";
    update_freq.sleep();
  }

  while (ros::ok())
  {
    current = ros::Time::now();
    robot.read(current, current-previous);
    controller_manager.update(current, current-previous);
    robot.write(current, current-previous); 
    previous = current;
    update_freq.sleep();
  }
}


