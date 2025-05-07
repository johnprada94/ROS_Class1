#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def dynamic_speed():
    # Publisher to '/cmd_vel' topic with message type Twist
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Initialize the node
    rospy.init_node('dynamic_speed', anonymous=True)
    
    # Set the initial speeds and toggle period (in seconds)
    slow_speed = 0.1
    fast_speed = 0.4
    toggle_period = rospy.get_param('~toggle_period', 5.0)  # Allow override via rosparam

    rate = rospy.Rate(10)  # 10 Hz loop rate
    start_time = rospy.get_time()
    # Initial state: start with slow speed
    use_slow_speed = True

    rospy.loginfo("Starting dynamic speed node. Toggling every %s seconds", toggle_period)

    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        elapsed = current_time - start_time
        
        # Toggle speed every toggle_period seconds
        if elapsed >= toggle_period:
            use_slow_speed = not use_slow_speed
            start_time = current_time  # reset timer for next toggle

        # Create a Twist message
        twist = Twist()
        twist.linear.x = slow_speed if use_slow_speed else fast_speed
        twist.angular.z = 0.0  # no rotation in this example
        
        # Publish the message
        pub.publish(twist)

        # Log the current state for debugging
        rospy.loginfo("Publishing speed: %.2f m/s", twist.linear.x)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        dynamic_speed()
    except rospy.ROSInterruptException:
        pass
