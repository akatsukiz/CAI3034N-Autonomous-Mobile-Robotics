#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import random
import math

"""Prey node for TurtleSim chase simulation.

Manages multiple prey turtles to avoid the predator and navigate the environment.
"""

class PreyTurtle:
    """Class to control an individual prey turtle."""

    def __init__(self, name, x, y, theta):
        """Initialise a prey turtle with a starting position.

        :param name: Name of the turtle (e.g., 'turtle2')
        :param x: Initial x-coordinate
        :param y: Initial y-coordinate
        :param theta: Initial orientation in radians
        """
        # Store the turtle's name
        self.name = name
        
        # Initialise state variables
        self.pose = Pose()         # Current pose of this turtle
        self.other_poses = {}      # Poses of other turtles (including predator)
        self.caught = False        # Flag indicating if caught by predator
        self.last_vel = Twist()    # Last velocity for smoothing
        
        # Override initial position with launch file parameters 
        x = rospy.get_param('/prey_node/{}_x'.format(name), x)
        y = rospy.get_param('/prey_node/{}_y'.format(name), y)

        # Spawn the turtle at the initial position
        rospy.wait_for_service('/spawn')  # Wait for spawn service to be available
        try:
            # Call the spawn service to create this turtle
            spawn = rospy.ServiceProxy('/spawn', Spawn)
            spawn(x, y, theta, name)
            rospy.loginfo("Spawned {} at ({}, {})".format(name, x, y))
            
            # Set ROS parameters to track this prey
            rospy.set_param('/prey/{}'.format(name), True)
            rospy.set_param('/caught/{}'.format(name), False)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to spawn {}: {}".format(name, e))
            return

        # Set up publisher for sending velocity commands to this turtle
        self.pub = rospy.Publisher('/{}/cmd_vel'.format(name), Twist, queue_size=10)
        
        # Subscribe to this turtle's pose updates
        rospy.Subscriber('/{}/pose'.format(name), Pose, self.pose_callback)

        # Subscribe to poses of all other turtles (for collision avoidance)
        for i in range(1, 6):  # turtle1 (predator) to turtle5
            other_name = 'turtle{}'.format(i)
            if other_name != self.name:
                try:
                    # Create a callback closure to capture the other turtle's name
                    def create_callback(name):
                        return lambda msg: self.other_pose_callback(msg, name)
                        
                    # Subscribe to other turtle's pose topic
                    rospy.Subscriber('/{}/pose'.format(other_name), Pose, create_callback(other_name))
                except rospy.ROSException as e:
                    rospy.logwarn("Error subscribing to {}: {}".format(other_name, e))

    def pose_callback(self, data):
        """Update this turtle's pose and caught status.

        :param data: Pose message from the topic
        """
        # Store the received pose data
        self.pose = data
        
        # Update caught status from ROS parameter
        self.caught = rospy.get_param('/caught/{}'.format(self.name), False)

    def other_pose_callback(self, data, other_name):
        """Update the pose of another turtle.

        :param data: Pose message from the topic
        :param other_name: Name of the other turtle
        """
        # Store the received pose data for the specific turtle
        self.other_poses[other_name] = data

    def distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points.

        :param x1, y1: Coordinates of the first point
        :param x2, y2: Coordinates of the second point
        :return: Distance between the points
        """
        # Standard Euclidean distance formula
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def smooth_velocity(self, target_vel):
        """Smooth velocity commands to reduce jerkiness.

        Uses exponential smoothing to blend new target velocity with previous velocity.

        :param target_vel: Desired Twist velocity
        :return: Smoothed Twist velocity
        """
        # Smoothing factor
        alpha = 0.3
        
        # Apply exponential smoothing to linear and angular velocities
        self.last_vel.linear.x = alpha * target_vel.linear.x + (1 - alpha) * self.last_vel.linear.x
        self.last_vel.angular.z = alpha * target_vel.angular.z + (1 - alpha) * self.last_vel.angular.z
        
        return self.last_vel

    def avoid_collisions(self, vel):
        """Adjust velocity to avoid collisions with other prey turtles.

        Only avoids uncaught prey - no need to avoid caught prey or predator.

        :param vel: Current Twist velocity
        :return: Adjusted Twist velocity
        """
        # Don't avoid collisions if already caught (controlled by predator)
        if self.caught:
            return vel
            
        # Distance threshold for collision avoidance
        avoidance_distance = 1.0
        
        # Check distance to each other turtle
        for other_name, other_pose in self.other_poses.items():
            # Skip predator (turtle1) and caught prey
            if other_name == 'turtle1' or rospy.get_param('/caught/{}'.format(other_name), False):
                continue
                
            # Calculate distance to other turtle
            dist = self.distance(self.pose.x, self.pose.y, other_pose.x, other_pose.y)
            
            # If too close, steer away
            if dist < avoidance_distance:
                # Calculate angle to other turtle
                angle_to_other = math.atan2(other_pose.y - self.pose.y, other_pose.x - self.pose.x)
                
                # Calculate angle to flee (opposite direction)
                flee_angle = (angle_to_other + math.pi) % (2 * math.pi)
                
                # Calculate angle difference (normalised to -pi to pi range)
                angle_diff = (flee_angle - self.pose.theta) % (2 * math.pi)
                if angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                    
                # Set velocity to steer away
                vel.angular.z = 1.5 * angle_diff
                vel.linear.x = 0.8
                
        return vel

    def avoid_borders(self, vel):
        """Adjust velocity to stay within simulation borders.

        Steers away from borders when getting too close.

        :param vel: Current Twist velocity
        :return: Adjusted Twist velocity
        """
        # Don't avoid borders if already caught (controlled by predator)
        if self.caught:
            return vel
            
        # Distance threshold from border to trigger avoidance
        border_margin = 1.5 #Larger margin than prey gives more reaction time for evasion.
        
        # Create a new velocity message to avoid modifying the input
        adjusted_vel = Twist()
        adjusted_vel.linear.x = vel.linear.x
        adjusted_vel.angular.z = vel.angular.z

        # Check proximity to left border (west)
        if self.pose.x < border_margin:
            target_angle = 0  # Turn east (away from border)
            # Calculate angle difference (normalised to -pi to pi range)
            angle_diff = (target_angle - self.pose.theta) % (2 * math.pi)
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            adjusted_vel.angular.z = 1.0 * angle_diff # Prey turns less aggressively than predator, smoother turns beneficial for evasion.
            adjusted_vel.linear.x = 0.8 # Reduce speed during avoidance, smaller speed reduction than predator to maintain some momentum for escape.
        
        # Check proximity to right border (east)
        elif self.pose.x > 11.0 - border_margin:
            target_angle = math.pi  # Turn west (away from border)
            # Calculate angle difference (normalised to -pi to pi range)
            angle_diff = (target_angle - self.pose.theta) % (2 * math.pi)
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            adjusted_vel.angular.z = 1.0 * angle_diff
            adjusted_vel.linear.x = 0.8
        
        # Check proximity to bottom border (south)
        elif self.pose.y < border_margin:
            target_angle = math.pi / 2  # Turn north (away from border)
            # Calculate angle difference (normalised to -pi to pi range)
            angle_diff = (target_angle - self.pose.theta) % (2 * math.pi)
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            adjusted_vel.angular.z = 1.0 * angle_diff
            adjusted_vel.linear.x = 0.8
        
        # Check proximity to top border (north)
        elif self.pose.y > 11.0 - border_margin:
            target_angle = 3 * math.pi / 2  # Turn south (away from border)
            # Calculate angle difference (normalised to -pi to pi range)
            angle_diff = (target_angle - self.pose.theta) % (2 * math.pi)
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            adjusted_vel.angular.z = 1.0 * angle_diff
            adjusted_vel.linear.x = 0.8

        return adjusted_vel

    def avoid_predator(self, vel):
        """Adjust velocity to flee from the predator if nearby.

        Increases speed and steers away when predator gets close.

        :param vel: Current Twist velocity
        :return: Adjusted Twist velocity
        """
        # Don't avoid predator if already caught (controlled by predator)
        if self.caught:
            return vel
            
        # Check if we have the predator's position
        if 'turtle1' in self.other_poses:
            predator = self.other_poses['turtle1']
            
            # Calculate distance to predator
            dist = self.distance(self.pose.x, self.pose.y, predator.x, predator.y)
            
            # If predator is nearby (within 3.0 units), flee
            if dist < 3.0:
                # Calculate angle to predator
                angle_to_pred = math.atan2(predator.y - self.pose.y, predator.x - self.pose.x)
                
                # Calculate angle to flee (opposite direction)
                flee_angle = (angle_to_pred + math.pi) % (2 * math.pi)
                
                # Calculate angle difference (normalised to -pi to pi range)
                angle_diff = (flee_angle - self.pose.theta) % (2 * math.pi)
                if angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                    
                # Set velocity to flee (turn away from predator)
                vel.angular.z = 2.0 * angle_diff
                
                # Speed up when closer to predator (max 1.5)
                vel.linear.x = min(1.5, 1.0 + (3.0 - dist) * 0.3)
                
                # Add slight randomness to make prey harder to predict
                vel.angular.z += random.uniform(-0.1, 0.1)
                
        return vel

    def move_once(self):
        """Compute and publish a single velocity command for this turtle.
        
        Combines different behaviours to determine prey movement:
        1. Default random movement
        2. Predator avoidance (highest priority)
        3. Border avoidance
        4. Collision avoidance with other prey
        """
        # If caught, don't control movement (predator will control)
        if self.caught:
            return

        # Base movement: constant speed with slight random turning
        vel = Twist()
        vel.linear.x = 0.7
        vel.angular.z = random.uniform(-0.2, 0.2)

        # Apply avoidance behaviours in priority order
        vel = self.avoid_predator(vel)   # 1. Flee from predator (highest priority)
        vel = self.avoid_borders(vel)    # 2. Stay within borders
        vel = self.avoid_collisions(vel) # 3. Avoid other prey

        # Smooth and publish the velocity
        vel = self.smooth_velocity(vel)
        self.pub.publish(vel)

class PreyNode:
    """Class to manage multiple prey turtles."""

    def __init__(self, prey_configs):
        """Initialise the PreyNode with a list of prey configurations.

        :param prey_configs: List of tuples (name, x, y, theta) for each prey
        """
        # Create PreyTurtle instances for each configuration
        self.preys = [PreyTurtle(name, x, y, theta) for name, x, y, theta in prey_configs]
        
        # Set control loop rate to 10Hz
        self.rate = rospy.Rate(10)

    def run(self):
        """Run the main loop to control all prey turtles."""
        while not rospy.is_shutdown():
            # Update movement for each prey turtle
            for prey in self.preys:
                prey.move_once()
                
            # Maintain loop rate
            self.rate.sleep()

if __name__ == '__main__':
    # Initialise the ROS node
    rospy.init_node('prey_node')
    
    # Default configurations (these will typically be overridden by launch file)
    prey_configs = [
        ('turtle2', 0.0, 0.0, 0.0),
        ('turtle3', 0.0, 0.0, 0.0),
        ('turtle4', 0.0, 0.0, 0.0),
        ('turtle5', 0.0, 0.0, 0.0)
    ]
    
    # Create prey node instance with the specified configurations
    prey_node = PreyNode(prey_configs)
    
    try:
        # Start the main control loop
        prey_node.run()
    except rospy.ROSInterruptException:
        pass