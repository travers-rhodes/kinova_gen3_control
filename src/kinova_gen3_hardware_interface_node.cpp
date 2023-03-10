#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <realtime_tools/realtime_publisher.h>
#include "std_msgs/Float32.h"

#include "kinova_gen3_control/kinova_gen3_hardware_interface.h"
#include "kinova_gen3_control/kinova_gen3_network_connection.h"
#include "kinova_gen3_control/fake_kinova_network_connection.h"
#include "kinova_gen3_control/gazebo_kinova_network_connection.h"

//https://slaterobots.com/blog/5abd8a1ed4442a651de5cb5b/how-to-implement-ros_control-on-a-custom-robot
int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinova_gen3_hardware_interface");
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  realtime_tools::RealtimePublisher<std_msgs::Float32> *realtime_pub = new
    realtime_tools::RealtimePublisher<std_msgs::Float32>(nh, "realtime_loop_rate", 4);

  std::cout << "Parsing URDF" << std::endl;
  urdf::Model urdf_robot;
  // load urdf from "robot_description"
  // TODO don't hard-code this string
  urdf_robot.initParam("robot_description");

  bool fake_connection;
  private_nh.getParam("fake_connection", fake_connection);
  bool use_gazebo;
  private_nh.getParam("use_gazebo", use_gazebo);

  std::shared_ptr<KinovaNetworkConnection> network_connection;
  if (fake_connection)
  {
    ROS_INFO("Creating fake network connection");
    network_connection = std::make_shared<FakeKinovaNetworkConnection>(); 
  }
  else if (use_gazebo) {
    ROS_INFO("Creating Gazebo network connection");
    network_connection = std::make_shared<GazeboKinovaNetworkConnection>(nh); 
  } 
  else
  {
    ROS_INFO("Creating real network connection");
    network_connection = std::make_shared<KinovaGen3NetworkConnection>(); 
  }

  ROS_INFO("Creating hardware interface");
  std::vector<std::string> joint_names = {
    "joint_1",
    "joint_2",
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
    const bool urdf_limits_ok = joint_limits_interface::getJointLimits(urdf_joint, limits);
    // Populate joint limits from the ros parameter server
    // Limits specified in the parameter server overwrite existing values in 'limits' 
    // Limits not specified in the parameter server preserve their existing values
    const bool rosparam_limits_ok = joint_limits_interface::getJointLimits(joint_names[i], nh, limits);
    limits_list.push_back(limits);
  }

  KinovaGen3HardwareInterface robot(
      joint_names,
      limits_list,
      network_connection);

  ROS_INFO("Starting controller manager");
  controller_manager::ControllerManager controller_manager(&robot, nh);
  ROS_INFO("Controller manager started");

  double control_loop_hz;
  if (!private_nh.getParam("control_loop_hz", control_loop_hz))
  {
    double default_control_loop_hz = 500;
    ROS_WARN("No control_loop_hz given in namespace: %s. Defaulting to %f.",
        private_nh.getNamespace().c_str(), default_control_loop_hz);
    control_loop_hz = default_control_loop_hz;
  }
    
  ros::Duration update_freq = ros::Duration(1.0/control_loop_hz);
  ros::Time previous = ros::Time::now();
  ros::Time current = ros::Time::now();

  while (ros::Time::now().toSec() == 0)
  {
    ROS_INFO_THROTTLE(0.5, "Waiting for clock to publish");
    update_freq.sleep();
  }

  ROS_INFO("Entering ros_control loop");
  double loop_time_secs = 0;
  double read_time_secs = 0;
  double update_time_secs = 0;
  double write_time_secs = 0;
  double sleep_time_secs = 0;
  double loop_count_mod_thousand = 0;
  ros::Time thousand_loops_start = ros::Time::now();
  ros::Time thousand_loops_end = ros::Time::now();
  ros::Duration controller_manager_loop_duration;
  ros::Time start = ros::Time::now();
  ros::Time stop = ros::Time::now();
  current = ros::Time::now();
  previous = current;
  // use a Rate loop to only sleep if we're faster than 1kHz (we generally aren't)
  ros::Rate loop_rate(update_freq);
  loop_rate.sleep();
  while (ros::ok())
  {
    start = stop;
    robot.read();
    stop = ros::Time::now();
    read_time_secs = (stop-start).toSec();

    start = stop;
    current = ros::Time::now();
    controller_manager_loop_duration = current-previous;
    if (controller_manager_loop_duration.toSec() == 0.00)
    {
      ROS_ERROR("KinovaGen3HardwareInterfaceNode: The loop duration was zero. I assume this was in simulation and that there's a bug. Total: %f sec. Read: %f; Update: %f; Write: %f; Sleep: %f", 
		    controller_manager_loop_duration.toSec(), read_time_secs, update_time_secs, write_time_secs, sleep_time_secs);
      continue;
    }
    controller_manager.update(current, controller_manager_loop_duration);
    // Warn the user if a loop ever takes 2 milliseconds or more
    if (controller_manager_loop_duration.toSec() > 2.0/control_loop_hz)
    {
      ROS_WARN_THROTTLE(0.1, "KinovaGen3HardwareInterfaceNode: The update loop took more than twice as long as expected. Total: %f sec. Read: %f; Update: %f; Write: %f; Sleep: %f", 
		    controller_manager_loop_duration.toSec(), read_time_secs, update_time_secs, write_time_secs, sleep_time_secs);
    }
    previous = current;
    stop = ros::Time::now();
    update_time_secs = (stop-start).toSec();

    start = stop;
    robot.write(controller_manager_loop_duration);
    stop = ros::Time::now();
    write_time_secs = (stop-start).toSec();

    start = stop;
    loop_rate.sleep();
    stop = ros::Time::now();
    sleep_time_secs = (stop-start).toSec();
    loop_count_mod_thousand++;
    if (loop_count_mod_thousand == 1000) {
      loop_count_mod_thousand = 0;
      thousand_loops_end = ros::Time::now();
      if (realtime_pub->trylock()){
        realtime_pub->msg_.data = 1000/(thousand_loops_end-thousand_loops_start).toSec();
        realtime_pub->unlockAndPublish();
      }
      thousand_loops_start = ros::Time::now();
    }
  }
}
