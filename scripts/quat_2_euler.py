#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion

pub = rospy.Publisher('rpy', Vector3, queue_size=10)
rpy = Vector3()
def callback(data): 
    euler = list(euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w]))
    rpy.x = euler[0]
    rpy.y = euler[1]
    rpy.z = euler[2]    
    pub.publish(rpy)


def quat_2_rpy():
    rospy.init_node('quat_2_rpy', anonymous=True)
    rospy.Subscriber("/imu", Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        quat_2_rpy()
    except rospy.ROSInterruptException:
        pass



