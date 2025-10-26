#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import threading
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Global variables for tracking positions
predator_pose = None
prey_poses = {}
captured_prey = []

def predator_pose_callback(data):
    """Update predator position."""
    global predator_pose
    predator_pose = data

def prey_pose_callback(data, prey_name):
    """Update prey position."""
    global prey_poses
    prey_poses[prey_name] = data

def get_distance(p1, p2):
    """Calculate Euclidean distance between two points."""
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

def find_closest_prey():
    """Find the nearest uncaptured prey."""
    global predator_pose, prey_poses, captured_prey

    if predator_pose is None or not prey_poses:
        return None, None

    closest_prey = None
    min_distance = float('inf')

    for prey_name, pose in prey_poses.items():
        if prey_name in captured_prey:
            continue  # Ignore captured prey

        distance = get_distance(predator_pose, pose)
        if distance < min_distance:
            min_distance = distance
            closest_prey = prey_name

    return closest_prey, min_distance

def stick_captured_prey(prey_name):
    """Keep captured prey synchronized with the predator."""
    pub = rospy.Publisher('/{}/cmd_vel'.format(prey_name), Twist, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if predator_pose:
            # Force prey to stay at the predator's location
            stick_cmd = Twist()
            stick_cmd.linear.x = 0.0
            stick_cmd.angular.z = 0.0
            pub.publish(stick_cmd)
        rate.sleep()

def chase_and_capture(pub):
    """Chase the closest prey and capture it."""
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        # Find the nearest uncaptured prey
        closest_prey, distance = find_closest_prey()

        # Exit if all prey are captured
        if closest_prey is None:
            rospy.loginfo("All prey captured! Mission complete.")
            break

        target_pose = prey_poses[closest_prey]

        # Movement control logic
        cmd = Twist()
        cmd.linear.x = min(2.0, distance)

        # Angle control
        angle_to_prey = math.atan2(target_pose.y - predator_pose.y, target_pose.x - predator_pose.x)
        angle_error = angle_to_prey - predator_pose.theta

        # Normalize angle error
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        cmd.angular.z = 4.0 * angle_error

        # Capture if close enough
        if distance < 0.5:
            rospy.loginfo("Captured: {}".format(closest_prey))
            captured_prey.append(closest_prey)

            # Keep the captured prey stuck to the predator
            t = threading.Thread(target=stick_captured_prey, args=(closest_prey,))
            t.start()

        pub.publish(cmd)
        rate.sleep()

def main():
    rospy.init_node('predator_turtle')

    # Set up predator (turtle1)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, predator_pose_callback)

    # Track all prey turtles
    prey_names = ["turtle2", "turtle3", "turtle4", "turtle5"]
    for prey in prey_names:
        rospy.Subscriber('/{}/pose'.format(prey), Pose, prey_pose_callback, prey)

    rospy.sleep(2)  # Allow time for setup

    rospy.loginfo("Predator is hunting!")
    chase_and_capture(pub)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
