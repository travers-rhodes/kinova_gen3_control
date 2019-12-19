#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include "kinova_gen3_control/kinova_gen3_hardware_interface.h"
#include "kinova_gen3_control/kinova_gen3_network_connection.h"

//https://slaterobots.com/blog/5abd8a1ed4442a651de5cb5b/how-to-implement-ros_control-on-a-custom-robot
int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinova_gen3_hardware_interface");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  KinovaGen3NetworkConnection kinova_gen3_connection;

  KinovaGen3HardwareInterface robot(kinova_gen3_connection.base_cyclic,
      kinova_gen3_connection.base,
      kinova_gen3_connection.actuator_config);

  controller_manager::ControllerManager controller_manager(&robot, nh);

  int loop_hz = 100;
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


