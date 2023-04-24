catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=1
catkin build
ln -s ../../build/inverse_kinematics_trajectory/compile_commands.json compile_commands.json
