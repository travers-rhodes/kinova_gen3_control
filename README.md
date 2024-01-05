# kinova_gen3_control
Open-source ros_control hardware_interface::RobotHW (hardware_interface::EffortJointInterface and hardware_interface::JointStateInterface) implementation for the Kinova Gen3 arm

## Installation
### Clone this repository to your catkin workspace.
```
cd ~
mkdir -p my_catkin_ws/src
cd my_catkin_ws/src
git clone https://github.com/travers-rhodes/kinova_gen3_control.git
```
### Clone the `kortex_description` package 
You can clone this from `https://github.com/Kinovarobotics/ros_kortex.git` (and just ignore the rest of the packages that are bundled with that).

```
cd ~/my_catkin_ws/src
git clone https://github.com/Kinovarobotics/ros_kortex.git
```


### Make sure you have all required dependencies installed
```
cd ~/my_catkin_ws/src
source /opt/ros/melodic/setup.bash
rosdep init
rosdep update
rosdep install --from-paths . --ignore-src -r
```

### Include the Kinova API
Please download the latest kortex_api zip folder listed at https://github.com/Kinovarobotics/kortex

Then unzip that file, then run something like the below to copy the needed lib and include files into new directories called kinova_gen3_control/kortex_api/linux_gcc_x86-64/lib and kinova_gen3_control/kortex_api/linux_gcc_x86-64/include
```
cd ~/Downloads
wget https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.3.0/linux_x86-64_x86_gcc.zip
unzip linux_x86-64_x86_gcc.zip -d linux_gcc_x86-64
cp -r ~/Downloads/linux_gcc_x86-64/* ~/my_catkin_ws/src/kinova_gen3_control/kortex_api/               
```

### Build the packages we care about (blacklist all the extra packages bundled with ros_kortex)
```
cd ~/my_catkin_ws
catkin config --blacklist kortex_control kortex_driver kortex_examples gazebo_version_helpers gazebo_grasp_plugin roboticsgroup_gazebo_plugins gen3_lite_gen3_lite_2f_move_it_config gen3_move_it_config gen3_robotiq_2f_140_move_it_config roboticsgroup_upatras_gazebo_plugins
source /opt/ros/melodic/setup.bash
catkin build
```

## TL;DR Installation
Together, all the install instructions above are
```
cd ~
mkdir -p my_catkin_ws/src
cd ~/my_catkin_ws/src
git clone https://github.com/travers-rhodes/kinova_gen3_control.git
git clone https://github.com/Kinovarobotics/ros_kortex.git
source /opt/ros/melodic/setup.bash
rosdep init
rosdep update
rosdep install --from-paths . --ignore-src -r
cd ~/Downloads
rm -f linux_x86-64_x86_gcc.zip
rm -rf linux_gcc_x86-64
wget https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.3.0/linux_x86-64_x86_gcc.zip
unzip linux_x86-64_x86_gcc.zip -d linux_gcc_x86-64
cp -r ~/Downloads/linux_gcc_x86-64/* ~/my_catkin_ws/src/kinova_gen3_control/kortex_api/ 
cd ~/my_catkin_ws
catkin config --blacklist kortex_control kortex_driver kortex_examples kortex_gazebo gazebo_version_helpers gazebo_grasp_plugin roboticsgroup_gazebo_plugins  gen3_lite_gen3_lite_2f_move_it_config gen3_move_it_config gen3_robotiq_2f_140_move_it_config roboticsgroup_upatras_gazebo_plugins
source /opt/ros/melodic/setup.bash
catkin build
source devel/setup.bash
```


## Usage
`roslaunch kinova_gen3_control default.launch` will 

1. Connect to the Kinova Gen3 arm (assumed to be at IP address `192.168.1.10`) using a TCP Transport Client on port 10000
1. Connect to the Kinova Gen3 arm using a UDP Transport Client on port 10001
1. Put the Robot in low-level control loop and load and start a `hardware_interface::EffortJointInterface` for each joint to send effort commands to each joint.
1. Load and start a `effort_controllers::JointPositionController` for each joint, which provides a ROS topic `/joint_X_position_controller/command` for each joint.
1. Start a `controller_manager::ControllerManager` that can be used to switch between high-level `ros_control::Controller`s. 
For example, to switch to a joint_trajectory_controller (the information for which is already loaded by `default.launch`) you can run the following from the command line:
```
rosservice call controller_manager/switch_controller "start_controllers: ['gen3_joint_trajectory_controller']
stop_controllers: ['joint_1_position_controller',
                   'joint_2_position_controller',
                   'joint_3_position_controller',
                   'joint_4_position_controller',
                   'joint_5_position_controller',
                   'joint_6_position_controller',
                   'joint_7_position_controller'
]
strictness: 1
start_asap: false
timeout: 0.0"
ok: True
```

## Debugging

If you want to test this package without a network connection to a robot, you can run
`roslaunch kinova_gen3_control default.launch fake_connection:=true`

## Realtime

Ideally, this node should be run on a realtime operating system, to maintain consistent timings between commands to the robot.
That's why the launch file runs this node on the `machine="control-computer"`. By default the `control-computer` machine is defined to be `localhost`,
but you can alternatively have this launch file run the node on a different machine by follwing the instructions at http://wiki.ros.org/roslaunch/XML/machine.

If you want the `control-computer` to be a different machine from `localhost`, you should set the arg `use_local_control_computer` to `false`.


