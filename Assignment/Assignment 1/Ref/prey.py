#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import random
import threading
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

# Global variable to track captured prey
captured_prey = []

def spawn_turtle(x, y, name):
    """Spawn a turtle at the specified position with a given name."""
    rospy.wait_for_service('/spawn')
    try:
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(x, y, 0, name)
        rospy.loginfo("Spawned {}".format(name))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

def move_prey(prey_name):
    """Move a prey turtle in random directions until captured."""
    pub = rospy.Publisher('/{}/cmd_vel'.format(prey_name), Twist, queue_size=10)
    rate = rospy.Rate(1)  # Move every second

    while not rospy.is_shutdown():
        if prey_name in captured_prey:
            # Stop the prey's movement once captured
            stop_cmd = Twist()
            pub.publish(stop_cmd)
            rospy.loginfo("{} is captured and has stopped.".format(prey_name))
            return

        # Random movement for active prey
        cmd = Twist()
        cmd.linear.x = random.uniform(0.5, 1.5)  # Random speed
        cmd.angular.z = random.uniform(-1.0, 1.0)  # Random turning
        pub.publish(cmd)
        rospy.loginfo("{} is moving".format(prey_name))
        rate.sleep()

def main():
    rospy.init_node('prey_turtles')

    # Define prey turtles and initial positions
    prey_names = ["turtle2", "turtle3", "turtle4", "turtle5"]
    positions = [(2, 2), (8, 8), (2, 8), (8, 2)]

    # Spawn multiple prey turtles
    for i, prey in enumerate(prey_names):
        spawn_turtle(positions[i][0], positions[i][1], prey)

    # Move each prey turtle in a separate thread
    for prey in prey_names:
        t = threading.Thread(target=move_prey, args=(prey,))
        t.start()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
