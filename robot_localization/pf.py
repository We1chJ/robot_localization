#!/usr/bin/env python3

""" This is the starter code for the robot localization project """

import rclpy
from threading import Thread
from rclpy.time import Time
from rclpy.node import Node
from numpy.random import random_sample
from copy import deepcopy
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import ParticleCloud, Particle
from nav2_msgs.msg import Particle as Nav2Particle
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from rclpy.duration import Duration
import math
import time
import numpy as np
from occupancy_field import OccupancyField
from helper_functions import TFHelper
from rclpy.qos import qos_profile_sensor_data
from angle_helpers import quaternion_from_euler

class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, w=1.0):
        """ Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of KeyboardInterruptthe hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized """ 
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        q = quaternion_from_euler(0, 0, self.theta)
        return Pose(position=Point(x=self.x, y=self.y, z=0.0),
                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))
    
    def predict(self, delta):
        """ Predict the next position based on the delta measured using
            the odometry """
        self.x += delta[0] * math.cos(self.theta) - delta[1] * math.sin(self.theta)
        self.y += delta[0] * math.sin(self.theta) + delta[1] * math.cos(self.theta)
        self.theta += delta[2]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi  # Normalize theta to [-pi, pi]

    # TODO: define additional helper functions if needed

class ParticleFilter(Node):
    """ The class that represents a Particle Filter ROS Node
        Attributes list:
            base_frame: the name of the robot base coordinate frame (should be "base_footprint" for most robots)
            map_frame: the name of the map coordinate frame (should be "map" in most cases)
            odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
            scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
            n_particles: the number of particles in the filter
            d_thresh: the amount of linear movement before triggering a filter update
            a_thresh: the amount of angular movement before triggering a filter update
            pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
            particle_pub: a publisher for the particle cloud
            last_scan_timestamp: this is used to keep track of the clock when using bags
            scan_to_process: the scan that our run_loop should process next
            occupancy_field: this helper class allows you to query the map for distance to closest obstacle
            transform_helper: this helps with various transform operations (abstracting away the tf2 module)
            particle_cloud: a list of particles representing a probability distribution over robot poses
            current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                                   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
            thread: this thread runs your main loop
    """
    def __init__(self):
        super().__init__('pf')
        self.base_frame = "base_footprint"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.scan_topic = "scan"        # the topic where we will get laser scans from 

        self.n_particles = 300          # the number of particles to use

        self.d_thresh = 0.2             # the amount of linear movement before performing an update
        self.a_thresh = math.pi/6       # the amount of angular movement before performing an update
        self.robot_pose = None
        self.odom_noise_rate = 0.1 # odom noise for pose prediction

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.update_initial_pose, 10)

        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = self.create_publisher(ParticleCloud, "particle_cloud", qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(Pose, "robot_pose", qos_profile_sensor_data)

        # laser_subscriber listens for data from the lidar
        self.create_subscription(LaserScan, self.scan_topic, self.scan_received, 10)

        # this is used to keep track of the timestamps coming from bag files
        # knowing this information helps us set the timestamp of our map -> odom
        # transform correctly
        self.last_scan_timestamp = None
        # this is the current scan that our run_loop should process
        self.scan_to_process = None
        # your particle cloud will go here
        self.particle_cloud = []

        self.current_odom_xy_theta = []
        self.occupancy_field : OccupancyField = OccupancyField(self)
        self.transform_helper = TFHelper(self)

        # we are using a thread to work around single threaded execution bottleneck
        thread = Thread(target=self.loop_wrapper)
        thread.start()
        self.transform_update_timer = self.create_timer(0.05, self.pub_latest_transform)

    def pub_latest_transform(self):
        """ This function takes care of sending out the map to odom transform """
        if self.last_scan_timestamp is None:
            return
        postdated_timestamp = Time.from_msg(self.last_scan_timestamp) + Duration(seconds=0.1)
        self.transform_helper.send_last_map_to_odom_transform(self.map_frame, self.odom_frame, postdated_timestamp)

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        while True:
            self.run_loop()
            time.sleep(0.1)

    def run_loop(self):
        """ This is the main run_loop of our particle filter.  It checks to see if
            any scans are ready and to be processed and will call several helper
            functions to complete the processing.
            
            You do not need to modify this function, but it is helpful to understand it.
        """
        if self.scan_to_process is None:
            return
        msg = self.scan_to_process

        (new_pose, delta_t) = self.transform_helper.get_matching_odom_pose(self.odom_frame,
                                                                           self.base_frame,
                                                                           msg.header.stamp)
        if new_pose is None:
            # we were unable to get the pose of the robot corresponding to the scan timestamp
            if delta_t is not None and delta_t < Duration(seconds=0.0):
                # we will never get this transform, since it is before our oldest one
                self.scan_to_process = None
            return
        
        (r, theta) = self.transform_helper.convert_scan_to_polar_in_robot_frame(msg, self.base_frame)
        print("r[0]={0}, theta[0]={1}".format(r[0], theta[0]))
        # clear the current scan so that we can process the next one
        self.scan_to_process = None

        self.odom_pose = new_pose
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        print("x: {0}, y: {1}, yaw: {2}".format(*new_odom_xy_theta))

        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
        elif not self.particle_cloud:
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud(msg.header.stamp)
        elif self.moved_far_enough_to_update(new_odom_xy_theta):
            # we have moved far enough to do an update!
            self.update_particles_with_odom()    # update based on odometry
            self.update_particles_with_laser(r, theta)   # update based on laser scan
            self.update_robot_pose()                # update robot's pose based on particles
            self.resample_particles()               # resample particles to focus on areas of high density
        # publish particles (so things like rviz can see them)
        self.publish_particles(msg.header.stamp)

    def moved_far_enough_to_update(self, new_odom_xy_theta):
        return math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh

    # From the one-dimensional example
    @staticmethod
    def weighted_values(values, probabilities, size):
        """ Return a random sample of size elements from the set values with the specified probabilities
            values: the values to sample from (numpy.ndarray)
            probabilities: the probability of selecting each element in values (numpy.ndarray)
            size: the number of samples
        """
        bins = np.add.accumulate(probabilities)
        indices = np.digitize(random_sample(size), bins)
        sample = []
        for ind in indices:
            sample.append(deepcopy(values[ind]))
        return sample

    def update_robot_pose(self):
        """ Update the estimate of the robot's pose given the updated particles.
            There are two logical methods for this:
                (1): compute the mean pose
                (2): compute the most likely pose (i.e. the mode of the distribution)
        """
        # first make sure that the particle weights are normalized
        self.normalize_particles()

        # TODO: assign the latest pose into self.robot_pose as a geometry_msgs.Pose object
        # just to get started we will fix the robot's pose to always be at the origin
        max_weight_particle = max(self.particle_cloud, key=lambda p: p.w)
        self.robot_pose = max_weight_particle.as_pose() # Compute the most likely pose (mode of the distribution)
        self.pose_pub.publish(self.robot_pose)

        if hasattr(self, 'odom_pose'):
            self.transform_helper.fix_map_to_odom_transform(self.robot_pose,
                                                            self.odom_pose)
        else:
            self.get_logger().warn("Can't set map->odom transform since no odom data received")

    def update_particles_with_odom(self):
        """ Update the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry.
        """
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        # TODO: modify particles using delta
        for p in self.particle_cloud:
            before_pose = deepcopy(p)
            p.predict(self.current_odom_xy_theta)
            
            self.add_noise_to_pose(p)
            
            assert (p.x != before_pose.x or p.y != before_pose.y or p.theta != before_pose.theta), \
                "Particle pose should change after adding noise"

    def resample_particles(self):
        """ Resample the particles according to the new particle weights.
            The weights stored with each particle should define the probability that a particular
            particle is selected in the resampling step.  You may want to make use of the given helper
            function draw_random_sample in helper_functions.py.
        """
        # make sure the distribution is normalized
        self.particle_cloud = ParticleFilter.weighted_values(self.particle_cloud, [p.weight for p in self.particle_cloud], len(self.particle_cloud))
        self.normalize_particles()

    def resample_particles_2d_distribution(self):
        """ Resample the particles according to the new particle weights.
            The weights stored with each particle should define the probability that a particular
            particle is selected in the resampling step.  You may want to make use of the given helper
            function draw_random_sample in helper_functions.py.
        """
        positions = np.array([(p.x, p.y) for p in self.particle_cloud])
        weights = np.array([p.w for p in self.particle_cloud])
        thetas = np.array([p.theta for p in self.particle_cloud])

        origin = self.occupancy_field.map.info.origin.position
        resolution = self.occupancy_field.map.info.resolution
        width = self.occupancy_field.map.info.width * resolution
        height = self.occupancy_field.map.info.height * resolution

        weight_map, x_edges, y_edges = np.histogram2d(
            positions[:, 0],
            positions[:, 1],
            bins=10,
            range=[
                [origin.x, origin.x + width],
                [origin.y, origin.y + height],
            ],
            weights=weights,
        )

        theta_map, x_edges, y_edges = np.histogram2d(
            positions[:, 0],
            positions[:, 1],
            bins=10,
            range=[
                [origin.x, origin.x + width],
                [origin.y, origin.y + height],
            ],
            weights=thetas,
        )

        # Normalize the weight map to create a probability distribution
        weight_map = weight_map.ravel()
        probs = weight_map / np.sum(weight_map)

        # Sample points on the 2D distribution defined by the weight map
        sampled_indexes = np.random.choice(len(probs), size=len(self.particle_cloud), p=probs)

        i_indices, j_indices = np.unravel_index(sampled_indexes, weight_map.shape)

        x_centers = 0.5 * (x_edges[:-1] + x_edges[1:])
        y_centers = 0.5 * (y_edges[:-1] + y_edges[1:])

        weights = weight_map[sampled_indexes]

        sampled_x_centers = x_centers[i_indices]
        sampled_y_centers = y_centers[j_indices]
        sampled_thetas = theta_map[sampled_indexes]

        self.particle_cloud = [
            Particle(x=sampled_x_centers[i], y=sampled_y_centers[i], theta=sampled_thetas[i], w=weights[i])
            for i in range(len(sampled_x_centers))]



    def update_particles_with_laser(self, r, theta):
        """ Updates the particle weights in response to the scan data
            r: the distance readings to obstacles
            theta: the angle relative to the robot frame for each corresponding reading 
        """
        assert len(r) == len(theta), "Length of r and theta must be the same"
        assert len(r) > 0, "r and theta must not be empty"
        assert all(d > 0 for d in r), "Distance readings must be positive"
        #assert all(t >= -math.pi and t <= math.pi for t in theta), "Angle readings must be between -pi and pi"
        
        shortest_distance = min(r)
        for p in self.particle_cloud:
            particle_shortest_distance = self.occupancy_field.get_closest_obstacle_distance(p.x, p.y)
            p.weight = 1/abs(shortest_distance - particle_shortest_distance + 1e-6)  # Add a small constant to avoid division by zero
            assert p.weight > 0, "Particle weight should be positive"
        self.normalize_particles()

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)

    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """ Initialize the particle cloud.
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is omitted, the odometry will be used """

        variation = [1,1,math.pi/4]  # the standard deviation of the noise to add to each particle
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
            variation = [2,2,math.pi/2]  # if we are using odometry to initialize the particles, we will use more noise
        self.particle_cloud = []

        particle_init_transform = np.random.normal(
            loc=xy_theta,
            scale=variation,
            size=(self.n_particles, 3)
        )
        for t in particle_init_transform:
            self.particle_cloud.append(Particle(x=t[0], y=t[1], theta=t[2], w=1))
        self.normalize_particles()

        assert len(self.particle_cloud) == self.n_particles, f"Particle cloud should have {self.n_particles} particles, but got {len(self.particle_cloud)}"
        self.update_robot_pose()

    def normalize_particles(self):
        """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
        sum_weights = sum(p.w for p in self.particle_cloud)
        for p in self.particle_cloud:
            p.w = p.w / sum_weights if sum_weights > 0 else 1.0 / len(self.particle_cloud)  # Avoid division by zero, assign equal weights if sum is zero
        # Check for negatives / improper sum
        sum_weights = sum(p.w for p in self.particle_cloud)
        assert all(p.w >= 0 for p in self.particle_cloud), "Particle weights should be non-negative"
        assert math.isclose(sum_weights, 1.0), f"Particle sums should add to 1, but got {sum_weights}"

    def publish_particles(self, timestamp):
        msg = ParticleCloud()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = timestamp
        for p in self.particle_cloud:
            msg.particles.append(Nav2Particle(pose=p.as_pose(), weight=p.w))
        self.particle_pub.publish(msg)

    def scan_received(self, msg):
        self.last_scan_timestamp = msg.header.stamp
        # we throw away scans until we are done processing the previous scan
        # self.scan_to_process is set to None in the run_loop 
        if self.scan_to_process is None:
            self.scan_to_process = msg

    def add_noise_to_pose(self, pose):
        """ Add noise to the given pose based on the odometry noise rate """
        pose.x += np.random.randn() * self.odom_noise_rate
        pose.y += np.random.randn() * self.odom_noise_rate
        pose.theta += np.random.randn() * self.odom_noise_rate

def main(args=None):
    rclpy.init()
    n = ParticleFilter()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
