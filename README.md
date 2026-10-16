# Commands to run for testing

`ros2 launch neato2_gazebo neato_gauntlet_world.py`

`ros2 launch robot_localization test_pf.py map_yaml:=robot_localization/maps/loc_map.yaml`

`rviz2 -d ~/ros2_ws/src/robot_localization/rviz/amcl.rviz`

`ros2 launch robot_localization test_pf.py   map_yaml:="/home/isa/Documents/Comp Robo/Project2/robot_localization/maps/mac_1st_floor_9_23.yaml"   use_sim_time:=true`
`IBGL_ALWAYS_SOFTWARE=1 rviz2   -d "/home/isa/Documents/Comp Robo/Project2/robot_localization/rviz/turtlebot_bag_files.rviz"   --ros-args -p use_sim_time:=true`
`ros2 bag play   "/home/isa/Documents/Comp Robo/Project2/robot_localization/bags/macfirst_floor_take_1"   --clock --delay`