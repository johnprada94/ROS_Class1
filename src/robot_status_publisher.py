#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ros_classes.msg import RobotStatus  # Import custom message
from std_msgs.msg import Float32

# Global variables for odometry and current twist (speed)
initial_pose = None
current_pose = None
current_speed = 0.0

def odom_callback(msg):
    global initial_pose, current_pose
    position = msg.pose.pose.position
    current_pose = position

    if initial_pose is None:
        initial_pose = position

def cmdvel_callback(msg):
    global current_speed
    # Update current_speed from the linear x component
    current_speed = abs(msg.linear.x)

def compute_distance():
    if initial_pose is None or current_pose is None:
        return 0.0
    dx = current_pose.x - initial_pose.x
    dy = current_pose.y - initial_pose.y
    return math.sqrt(dx**2 + dy**2)

def robot_status_publisher():
    rospy.init_node('robot_status_publisher', anonymous=True)

    # Subscribers for odometry and cmd_vel topics
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/cmd_vel', Twist, cmdvel_callback)

    # Publisher for the custom RobotStatus message
    status_pub = rospy.Publisher('/robot_status', RobotStatus, queue_size=10)

    rate = rospy.Rate(1)  # Publish status at 1 Hz
    rospy.loginfo("Robot Status Publisher Node started.")

    while not rospy.is_shutdown():
        distance = compute_distance()

        # Determine status message based on current speed
        if current_speed < 0.25:
            status = "Moving slowly"
        else:
            status = "Moving fast"

        status_msg = RobotStatus()
        status_msg.status_message = status
        status_msg.traveled_distance = distance
        status_msg.current_speed = current_speed

        # Publish the status message
        status_pub.publish(status_msg)
        rospy.loginfo("Robot Status: %s | Distance: %.2f m | Speed: %.2f m/s", status, distance, current_speed)
        rate.sleep()

if __name__ == '__main__':
    try:
        robot_status_publisher()
    except rospy.ROSInterruptException:
        pass
