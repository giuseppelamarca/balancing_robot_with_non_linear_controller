#!/usr/bin/env python
import rospy
import rosservice
import time
import sys
from geometry_msgs.msg import Twist, Vector3
from std_srvs.srv import Empty, EmptyRequest

pitch = 0 

def callback(rpy):
    global pitch
    pitch = rpy.y


def controller():
    rospy.init_node('test', anonymous=True)
    ctrl_pub = Twist()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/rpy', Vector3, callback)
    rospy.wait_for_service("/gazebo/reset_world")
    ser = rospy.ServiceProxy("/gazebo/reset_world", Empty)
    vel_i = float(sys.argv[1])
    vel = vel_i
    while not rospy.is_shutdown():
        ser(EmptyRequest())
        rospy.sleep(5)
        if pitch<0:
            vel = -vel_i
        ctrl_pub.linear.x = -vel
        pub.publish(ctrl_pub)
        rospy.sleep(5)
        ctrl_pub.linear.x = vel
        pub.publish(ctrl_pub)
        rospy.sleep(5)
        ctrl_pub.linear.x = 0
        pub.publish(ctrl_pub)
        rospy.sleep(5)

            


if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
