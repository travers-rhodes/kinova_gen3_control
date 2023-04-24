#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "mock_position_joint_hardware_interface/mocked_position_joint_hardware_interface.h"


//https://slaterobots.com/blog/5abd8a1ed4442a651de5cb5b/how-to-implement-ros_control-on-a-custom-robot
int main(int argc, char** argv)
{
  ros::init(argc, argv, "mocked_gen3_hardware_interface");
  ros::NodeHandle nh;
  // need this in order to have a thread for controller_manager to run on
  ros::AsyncSpinner spinner(1);
  spinner.start();

  MockedPositionInterface robot;

  ROS_INFO("Starting controller manager");
  controller_manager::ControllerManager cm(&robot, nh);
  ROS_INFO("Controller manager started");

  ROS_INFO("Entering ros_control loop");
  ros::Rate loop_rate(20);
  ros::Time previous = ros::Time::now();
  ros::Time current = ros::Time::now();
  ros::Duration controller_manager_loop_duration;
  while (ros::ok())
  {
    robot.read();
    current = ros::Time::now();
    controller_manager_loop_duration = current-previous;
    cm.update(current, controller_manager_loop_duration);
    robot.write(controller_manager_loop_duration);
    loop_rate.sleep();
  }
}
