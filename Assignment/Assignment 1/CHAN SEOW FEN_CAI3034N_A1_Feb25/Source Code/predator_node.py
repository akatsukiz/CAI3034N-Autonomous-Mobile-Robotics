#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

"""Predator node for TurtleSim chase simulation.

Controls the predator turtle (turtle1) to chase and catch prey turtles.
"""

class PredatorTurtle:
    """Class to manage the predator turtle's behaviour."""

    def __init__(self):
        """Initialise the PredatorTurtle node.

        Sets up the ROS node, initialises variables, and configures publishers/subscribers.
        """
        # Initialise the ROS node with name 'predator_node'
        rospy.init_node('predator_node')
        
        # State variables
        self.game_ended = False  # Flag to indicate if all prey are caught
        self.name = 'turtle1'    # Name of the predator turtle
        self.pose = Pose()       # Current pose of the predator
        self.prey_poses = {}     # Dictionary of prey names to their poses
        self.caught_prey = []    # List of caught prey names
        
        # Velocity smoothing variables
        self.last_vel = Twist()  # Last velocity for smoothing predator movement
        self.last_vel_prey = {}  # Last velocities for smoothing caught prey movements

        # Publisher for predator velocity commands
        self.pub = rospy.Publisher('/{}/cmd_vel'.format(self.name), Twist, queue_size=10)
        
        # Subscriber for predator's pose updates
        rospy.Subscriber('/{}/pose'.format(self.name), Pose, self.pose_callback)
        
        # Set control loop rate to 10Hz
        self.rate = rospy.Rate(10)
        
        # Dictionary to store publishers for controlling caught prey
        self.caught_pubs = {}

        # Initialise 'caught' parameters for all possible prey (turtle2 to turtle5)
        # These parameters will be used to track which prey have been caught
        for i in range(2, 6):  # Match the 4 prey from the launch file
            prey_name = 'turtle{}'.format(i)
            if not rospy.has_param('/caught/{}'.format(prey_name)):
                rospy.set_param('/caught/{}'.format(prey_name), False)

        # Delay to give prey a head start before predator begins hunting
        rospy.loginfo("Giving prey a head start of 5 seconds...")
        time.sleep(5)

    def pose_callback(self, data):
        """Update the predator's current pose.

        :param data: Pose message from the topic
        """
        # Store the received pose data
        self.pose = data

    def prey_pose_callback(self, data, prey_name):
        """Update the pose of a specific prey turtle.

        :param data: Pose message from the topic
        :param prey_name: Name of the prey turtle
        """
        # Store the received pose data for the specific prey
        self.prey_poses[prey_name] = data

    def predict_position(self, prey_pose):
        """Predict a prey's future position based on its current velocity.

        Uses a simple linear prediction based on current velocity vector.

        :param prey_pose: Current pose of the prey
        :return: Tuple (x, y) of predicted position
        """
        # Time horizon for prediction (in seconds)
        time_step = 0.5
        
        # Calculate predicted position using current velocity and direction
        pred_x = prey_pose.x + prey_pose.linear_velocity * math.cos(prey_pose.theta) * time_step
        pred_y = prey_pose.y + prey_pose.linear_velocity * math.sin(prey_pose.theta) * time_step
        
        return pred_x, pred_y

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
        # Smoothing factor (0 to 1, higher = less smoothing)
        alpha = 0.3
        
        # Apply exponential smoothing to linear and angular velocities
        self.last_vel.linear.x = alpha * target_vel.linear.x + (1 - alpha) * self.last_vel.linear.x
        self.last_vel.angular.z = alpha * target_vel.angular.z + (1 - alpha) * self.last_vel.angular.z
        
        return self.last_vel

    def avoid_borders(self, vel):
        """Adjust velocity to prevent hitting simulation borders.

        Checks proximity to borders and adjusts direction to avoid collisions.

        :param vel: Current Twist velocity
        :return: Adjusted Twist velocity
        """
        # Distance from border to start avoidance behaviour
        border_margin = 1.0 # Smaller margin than prey as the predator's primary goal is pursuit
        
        
        # Create a new velocity message to avoid modifying the input
        adjusted_vel = Twist()
        adjusted_vel.linear.x = vel.linear.x
        adjusted_vel.angular.z = vel.angular.z

        # Check proximity to left border (west)
        if self.pose.x < 1 + border_margin:
            target_angle = 0  # Turn east (away from border)
            # Calculate angle difference (normalised to -pi to pi range)
            angle_diff = (target_angle - self.pose.theta) % (2 * math.pi)
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            adjusted_vel.angular.z = 2.0 * angle_diff # Higher angular velocity scaling than prey for quicker turns during pursuit.
            adjusted_vel.linear.x = 0.5  # Reduce speed during avoidance, greater speed reduction than prey to ensure sharper turns and prevent overshooting
        
        # Check proximity to right border (east)
        elif self.pose.x > 10 - border_margin:
            target_angle = math.pi  # Turn west (away from border)
            # Calculate angle difference (normalised to -pi to pi range)
            angle_diff = (target_angle - self.pose.theta) % (2 * math.pi)
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            adjusted_vel.angular.z = 2.0 * angle_diff
            adjusted_vel.linear.x = 0.5
        
        # Check proximity to bottom border (south)
        elif self.pose.y < 1 + border_margin:
            target_angle = math.pi / 2  # Turn north (away from border)
            # Calculate angle difference (normalised to -pi to pi range)
            angle_diff = (target_angle - self.pose.theta) % (2 * math.pi)
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            adjusted_vel.angular.z = 2.0 * angle_diff
            adjusted_vel.linear.x = 0.5
        
        # Check proximity to top border (north)
        elif self.pose.y > 10 - border_margin:
            target_angle = 3 * math.pi / 2  # Turn south (away from border)
            # Calculate angle difference (normalised to -pi to pi range)
            angle_diff = (target_angle - self.pose.theta) % (2 * math.pi)
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            adjusted_vel.angular.z = 2.0 * angle_diff
            adjusted_vel.linear.x = 0.5

        return adjusted_vel

    def sync_caught_prey(self, all_caught=False):
        """Synchronise caught prey to follow the predator in a line.

        Makes caught prey turtles follow behind the predator in a 'conga line'.

        :param all_caught: True if all prey are caught, enabling more precise control
        """
        # Loop through each caught prey in order
        for i, prey_name in enumerate(self.caught_prey):
            # Skip if we don't have a publisher for this prey
            if prey_name not in self.caught_pubs:
                continue
                
            # Initialise velocity command for this prey
            follow_vel = Twist()
            
            # Calculate desired position: each prey should be positioned 
            # behind the predator, spaced by index in the caught list
            distance_behind = 0.35 * (i + 1)  # Each prey spaced 0.35 units apart
            
            # Calculate target position behind predator based on predator's orientation
            desired_x = self.pose.x - distance_behind * math.cos(self.pose.theta)
            desired_y = self.pose.y - distance_behind * math.sin(self.pose.theta)
            
            # Get current prey position (if available)
            prey_pose = self.prey_poses.get(prey_name)
            if not prey_pose:
                continue

            # Calculate movement parameters towards desired position
            dist = self.distance(prey_pose.x, prey_pose.y, desired_x, desired_y)
            angle = math.atan2(desired_y - prey_pose.y, desired_x - prey_pose.x)
            
            # Calculate angle difference (normalised to -pi to pi range)
            angle_diff = (angle - prey_pose.theta) % (2 * math.pi)
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi

            # Adjust velocity based on whether all prey are caught
            # When all prey are caught, more precise control is used
            if all_caught:
                # Stronger control when all prey are caught for tighter formation
                follow_vel.linear.x = 4.0 * dist
                follow_vel.angular.z = 5.0 * angle_diff
                
                # Stop when very close to desired position
                if dist < 0.1:
                    follow_vel.linear.x = 0.0
                    follow_vel.angular.z = 0.0
            else:
                # More relaxed control when still hunting
                follow_vel.linear.x = 2.5 * dist
                follow_vel.angular.z = 3.5 * angle_diff

            # Apply velocity smoothing to reduce jerkiness
            if prey_name in self.last_vel_prey:
                alpha = 0.4  # Smoothing factor
                self.last_vel_prey[prey_name].linear.x = alpha * follow_vel.linear.x + (1 - alpha) * self.last_vel_prey[prey_name].linear.x
                self.last_vel_prey[prey_name].angular.z = alpha * follow_vel.angular.z + (1 - alpha) * self.last_vel_prey[prey_name].angular.z
                follow_vel = self.last_vel_prey[prey_name]
            else:
                # Initialise smoothing for new caught prey
                self.last_vel_prey[prey_name] = follow_vel

            # Send command to the prey
            self.caught_pubs[prey_name].publish(follow_vel)

    def chase(self):
        """Execute the main chase loop to pursue and catch prey.
        
        This is the primary method that runs continuously to:
        1. Track prey positions
        2. Chase the nearest uncaught prey
        3. Capture prey when close enough
        4. Make caught prey follow in a line
        """
        # Reset velocity smoothing for caught prey
        self.last_vel_prey = {}

        # Step 1: Subscribe to all prey poses dynamically
        prey_count = 0
        for i in range(2, 6):  # Support 4 prey (turtle2 to turtle5)
            prey_name = 'turtle{}'.format(i)
            try:
                # Check if this prey exists in parameters
                if rospy.has_param('/prey/{}'.format(prey_name)):
                    prey_count += 1
                    # Create a callback closure to capture the prey name
                    def create_callback(name):
                        return lambda msg: self.prey_pose_callback(msg, name)
                        
                    # Subscribe to this prey's pose topic
                    rospy.Subscriber('/{}/pose'.format(prey_name), Pose, create_callback(prey_name))
                    
                    # Create publisher for controlling this prey (when caught)
                    if prey_name not in self.caught_pubs:
                        self.caught_pubs[prey_name] = rospy.Publisher('/{}/cmd_vel'.format(prey_name), Twist, queue_size=10)
            except rospy.ROSException as e:
                rospy.logwarn("Error subscribing to {}: {}".format(prey_name, e))
                continue
                
        rospy.loginfo("Found {} prey turtles to chase".format(prey_count))
        
        # Step 2: Wait for initial prey positions before starting chase
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while len(self.prey_poses) < prey_count and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
            
        # Warn if not all prey positions were detected
        if len(self.prey_poses) < prey_count:
            rospy.logwarn("Could only detect {} prey positions".format(len(self.prey_poses)))

        rospy.loginfo("Predator start hunting!")

        # Step 3: Main chase loop
        while not rospy.is_shutdown():
            # Check if all prey are caught
            all_caught = len(self.prey_poses) > 0 and all(prey_name in self.caught_prey for prey_name in self.prey_poses)
              
            # If all prey are caught and not already marked as game ended
            if all_caught and self.caught_prey and not self.game_ended:
                rospy.loginfo("All prey turtles have been caught, game ended.")
                self.game_ended = True
            
            # Synchronise caught prey positions to follow predator
            if self.caught_prey:
                self.sync_caught_prey(all_caught)

            # If all prey are caught, stop the predator
            if all_caught and self.caught_prey:
                self.pub.publish(Twist())  # Zero velocity
                self.rate.sleep()
                continue

            # Wait if no prey poses have been received yet
            if not self.prey_poses:
                rospy.loginfo("No prey poses received yet")
                rospy.sleep(1.0)
                continue

            # Find nearest uncaught prey (using predicted position)
            target = None
            min_dist = float('inf')
            for prey_name, pose in self.prey_poses.items():
                if prey_name not in self.caught_prey:
                    # Predict where this prey will be
                    pred_x, pred_y = self.predict_position(pose)
                    # Calculate distance to predicted position
                    dist = self.distance(self.pose.x, self.pose.y, pred_x, pred_y)
                    # Keep track of nearest prey
                    if dist < min_dist:
                        min_dist = dist
                        target = (prey_name, pred_x, pred_y)

            # If we found a target prey
            if target:
                prey_name, pred_x, pred_y = target
                current_prey_pose = self.prey_poses[prey_name]

                # Check if prey is within catching distance
                current_dist = self.distance(self.pose.x, self.pose.y, current_prey_pose.x, current_prey_pose.y)
                if current_dist < 0.6:  # Catching distance threshold
                    # Calculate angle to prey
                    angle_to_prey = math.atan2(current_prey_pose.y - self.pose.y, current_prey_pose.x - self.pose.x)
                    
                    # Check if prey is within predator's field of view
                    angle_diff = abs((angle_to_prey - self.pose.theta) % (2 * math.pi))
                    if angle_diff > math.pi:
                        angle_diff = 2 * math.pi - angle_diff
                        
                    # Can only catch prey if they're in front of the predator (within 90 degree field of view)
                    if angle_diff < math.pi / 2:
                        # Mark prey as caught
                        self.caught_prey.append(prey_name)
                        rospy.set_param('/caught/{}'.format(prey_name), True)
                        rospy.loginfo("Caught {}".format(prey_name))
                            
                        # Initialise velocity smoothing for caught prey
                        self.last_vel_prey[prey_name] = Twist()
                    continue

                # Calculate velocity towards predicted prey position
                vel = Twist()
                
                # Calculate angle towards predicted position
                angle = math.atan2(pred_y - self.pose.y, pred_x - self.pose.x)
                
                # Calculate angle difference (normalised to -pi to pi range)
                angle_diff = (angle - self.pose.theta) % (2 * math.pi)
                if angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                    
                # Set linear velocity proportional to distance (capped at 1.2)
                vel.linear.x = min(1.2, 1.0 * min_dist)
                
                # Set angular velocity proportional to angle difference
                vel.angular.z = 3.0 * angle_diff

                # Apply border avoidance and velocity smoothing
                vel = self.avoid_borders(vel)
                vel = self.smooth_velocity(vel)
                
                # Send velocity command to predator
                self.pub.publish(vel)

            # Maintain loop rate
            self.rate.sleep()

if __name__ == '__main__':
    # Create predator instance
    predator = PredatorTurtle()
    try:
        # Start chase loop
        predator.chase()
    except rospy.ROSInterruptException:
        pass