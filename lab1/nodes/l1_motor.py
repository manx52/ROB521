#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def publisher_node():
    """TODO: initialize the publisher node here, \
            and publish wheel command to the cmd_vel topic')"""
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    twist = Twist()
    twist.linear.x = 0.1
    twist.angular.z = 0.1
    cmd_pub.publish(twist)
def main():
    try:
        rospy.init_node('motor')
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            publisher_node()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
