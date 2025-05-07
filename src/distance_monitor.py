#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyResponse  # For Exercise 4 (service)

# Global variables to store initial pose and current position
initial_pose = None
current_pose = None

def odom_callback(msg):
    global initial_pose, current_pose
    # Extract the x, y position from the odometry message
    position = msg.pose.pose.position
    current_pose = position

    # Set the initial_pose if it hasn't been set yet
    if initial_pose is None:
        initial_pose = position

def reset_distance_callback(req):
    """Service callback to reset the starting point to current position."""
    global initial_pose, current_pose
    if current_pose is not None:
        initial_pose = current_pose
        rospy.loginfo("Distance has been reset. New starting point: (%.2f, %.2f)", initial_pose.x, initial_pose.y)
    else:
        rospy.logwarn("Current pose not available to reset distance.")
    return EmptyResponse()

def distance_monitor():
    rospy.init_node('distance_monitor', anonymous=True)

    # Subscriber to /odom topic
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Publisher to /traveled_distance topic
    distance_pub = rospy.Publisher('/traveled_distance', Float32, queue_size=10)

    # Service to reset the traveled distance (see Exercise 4)
    rospy.Service('/reset_distance', Empty, reset_distance_callback)

    rate = rospy.Rate(1)  # Publish distance at 1 Hz
    rospy.loginfo("Distance Monitor Node started, publishing at 1 Hz.")

    while not rospy.is_shutdown():
        if initial_pose is None or current_pose is None:
            # Wait until the first odometry message is received
            rospy.logdebug("Waiting for odometry data...")
            rate.sleep()
            continue

        # Calculate Euclidean distance from initial position to current position
        dx = current_pose.x - initial_pose.x
        dy = current_pose.y - initial_pose.y
        distance = math.sqrt(dx**2 + dy**2)

        # Publish the distance
        distance_pub.publish(distance)
        rospy.loginfo("Traveled distance: %.3f m", distance)
        rate.sleep()

if __name__ == '__main__':
    try:
        distance_monitor()
    except rospy.ROSInterruptException:
        pass
