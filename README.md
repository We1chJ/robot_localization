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

Initially, we adopted the intuitive and simple approach of taking the pose of the particle with the highest weight. With this approach, the benefit is that it is easy to implement and every robot pose update will be a valid pose in the real world, instead of potentially being in the walls or other unrealistic locations. However, the drawback is that the algorithm can easily lock onto an incorrect particle at a given moment, and swing all future particle resampling and pose estimation to be clustered around a falsely identified particle pose. For example, the initial robot pose will be completely random upon the initialization of particle clouds, which can be significantly off from the true starting position. This offset can lead to further drifting in the pose estimation in the future that can't be easily self-corrected. In addition, in the visualization, the robot pose will also frequently glitch and teleport, which is hard to debug and grasp.

To avoid incorrectly locking onto a poor particle pose forever, we decided to finally adopt the approach of taking the weighted average of all particles' poses on every update. This approach not only makes the estimated robot pose update much more smoothly, but also allows a higher tolerance for multiple clusters of ambiguous guesses, which makes future self-correction possible and easier. Although this approach can potentially create invalid robot poses and takes longer to arrive at a good estimate, the robot is entirely controlled manually in the context of this project and therefore temporarily receiving invalid pose estimates is more acceptable. However, in any other context where the robots constantly perform actions based on their state, invalid poses from this approach need to be handled more carefully to avoid undefined behavior in the real world.


### 3.2 Robot Kidnapping Problem 

We are interested in exploring the classic robot kidnapping problem in this section, consisting of two parts. 

### 3.2.1 Unknown Initial Position

In this project, the robot never has knowledge of its true initial location. But for the purpose of a global localization algorithm like this particle filter, the robot should be able to estimate where it is in a known map environment eventually over time. Therefore, we designed our algorithm under the constraint of an unknown initial position. 

As stated in Section 3.1, when we used the max weight method to update robot pose, the performance of the localization is hindered significantly by the ambiguity of initial positions. During initialization, the robot is frequently locked onto a random pose at first, and it often can't self-correct itself to produce an estimate closer to the true pose. 

This issue is effectively resolved once we switched the robot pose update strategy to the weighted average approach, where the robot pose won't be locked onto a single pose easily. With this greater tolerance and flexibility in pose estimation, the particle filter algorithm is able to effectively produce an accurate estimate close to the true pose over time. The robot pose can drift and shift drastically within the first few seconds of the simulation when the distribution of the particles is relatively spread out. However, once the filter has taken in enough details of scan data over time, particles quickly collapse into a reasonable estimated pose, effectively overcoming the uncertainty in initial position.

### 3.2.2 Kidnapping Recovery

Another potential issue under the robot kidnapping category is that, when the change of a robot's true pose isn't reflected in the sensor readings at all for a duration of time, all future estimation will drift and be offset without knowing it. This problem can occur due to unexpected impact from the surroundings, or uncaptured movements due to lossy sensor connections. When the estimation and the reality differ significantly, the robot can start making decisions based on falsely confident pose estimation and encounter undefined behavior.

In the original implementation, the particle filter won't be able to recover from large, sudden shifts in particle estimation because the majority of particles have already converged to be near the true pose. At this point, resampling won't be able to help the robot escape the incorrect pose because there are no other potential particles to try.

To be able to recover from an unexpected offset, the robot needs to first identify it. In our particle filter implementation, when calculating the weight for a particle, it computes a likelihood value first based on how well the theoretical raycasting matches with the actual lidar scan. We can store this likelihood value for every particle, and take the mean of them. If the mean likelihood value is below a certain threshold, the algorithm can claim that the robot is kidnapped with a large offset. 

Once identified, we will intentionally inject by overwriting a fraction of the current particles to be completely randomly distributed across the entire map. This alone allows the algorithm to have enough possibility to catch onto another single particle that is closer to the new true pose. As more data and update cycles go by, this single particle will gather more and more weights until it is enough to correct the pose estimation.


## 4. Challenges & Future Improvements

invalid lidar scan reading (bounded by min, max range)

particle initializeations out of bounds of maps at invalid spots

pose referecne frame conversion, keeping it standard

## 5. Lessons Learned
