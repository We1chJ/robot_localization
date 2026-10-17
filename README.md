# Commands to run for testing

`ros2 launch neato2_gazebo neato_gauntlet_world.py`

`ros2 launch robot_localization test_pf.py map_yaml:=robot_localization/maps/gauntlet.yaml use_sim_time:=true`

`rviz2 -d robot_localization/rviz/amcl.rviz --ros-args -p use_sim_time:=true`

# Mac run

`ros2 launch robot_localization test_pf.py map_yaml:="./robot_localization/maps/mac_1st_floor_9_23.yaml" use_sim_time:=true`

`IBGL_ALWAYS_SOFTWARE=1 rviz2 -d "./robot_localization/rviz/turtlebot_bag_files.rviz" --ros-args -p use_sim_time:=true`

`ros2 bag play "./robot_localization/bags/macfirst_floor_take_1" --clock`

# Robot Localization

Author: Jack, Isa

For Olin ENGR3590 Computational Introduction to Robotics

This README is a comprehensive report that contains both conceptual and
programmatic information regarding implementation details of our Lazy Neato
project. For clarity, the parameters detailed below were from our initial
testing; however, for certain gifs and bag recordings, we modified them to
better show the individual behavior.

## 1. Project Overview

The goal of this project is to learn and practice the implementation of a
specific type of Bayesian filters, particle filters, as well as examining the
effects of many specific design decisions and advanced techniques on the task of
robot localization. This writeup will present details on the overall approach of
our implementation of particle filters, small design adjustments we made on the
algorithm, learnings from debuggings, and additional techniques, such as particle injections.

## 2. Particle Filters

Our implementation of the particle filters follows a well-structured architecture:

(talk about classes, interactions, critical functions, etc. briefly)

## 3. Design Decisions

In this section, we document some of the important design decisions we made in our particle filter implementation.

### 3.1 Robot Pose Update Method

One of the last steps of the particle filter sensing loop is to determine the estimated robot pose based on the calculated particle cloud. With all the particles carrying a certain weight representing how likely they are to be the true robot pose, we have the flexibility to experiment with different methods of updating the robot pose. 

Initially, we adopted the intuitive and simple approach of taking the pose of the particle with the highest weight. With this approach, the benefit is that it is easy to implement and every robot pose update will be a valid pose in the real world, instead of potentially being in the walls or other unrealistic locations. However, the drawback is that the algorithm can easily lock onto an incorrect particle at a given moment, and swing all future particle resampling and pose estimation to be clustered around a falsely identified particle pose. On the visualization, the robot pose will also frequently glitch and teleport, which is hard to debug and grasp.

To avoid incorrectly locking onto a poor particle pose forever, we decided to finally adopt the approach of taking the weighted average of all particles' poses on every update. This approach not only makes the estimated robot pose update much more smoothly, but also allows a higher tolerance for multiple clusters of ambiguous guesses, which makes future self-correction possible and easier. Although this approach can potentially create invalid robot poses, the robot is entirely controlled manually in the context of this project and therefore temporarily receiving invalid pose estimates is more acceptable. However, in any other context where the robots constantly perform actions heavily based on its pose estimation, invalid poses from this approach need to be handled more carefully to avoid undefined behavior in the real world.


### 3.2 Robot Kidnapping Problem 

Unknown initial position

kidnapping recovery (talk about particle injection)

## 4. Challenges & Future Improvements

invalid lidar scan reading (bounded by min, max range)

particle initializeations out of bounds of maps at invalid spots

## 5. Lessons Learned
