#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetModelState,GetModelStateRequest
import numpy as np
from nav_msgs.msg import Odometry

pitch = 0
e_old = 0
e_int = 0
em_old = 0
em_int = 0
old_time = 0

def reset_simulation():
    global e_int, e_old, em_int, em_old, old_time
    e_int = 0
    e_old = 0
    em_old = 0
    em_int = 0


def callback(rpy):
    global pitch
    pitch = rpy.y    

def odom_callback(o):
    global odom
    odom = o.pose.pose.position.x

def controller():
    global e_int, e_old, em_int, em_old, old_time
    rospy.init_node('pid_controller', anonymous=True)
    ctrl_pub = Twist()

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    error_pub = rospy.Publisher("/error_angle", Float64, queue_size=1)

    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.Subscriber("rpy", Vector3, callback)

    rospy.wait_for_service("/gazebo/reset_world")
    rospy.wait_for_service("/gazebo/get_model_state")
    gazebo_service = rospy.ServiceProxy("/gazebo/reset_world", Empty)
    robot_pose = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    old_time = rospy.get_time()
    rospy.sleep(0.5)
    while not rospy.is_shutdown():
        '''
        # check if the robot is moving too faraway (remove when the robot is perfect balanced
        model = GetModelStateRequest()
        model.model_name = "self_balancing_robot"
        model.relative_entity_name = ""
        pose = robot_pose(model).pose.position
        if np.sqrt(pose.x * pose.x + pose.y * pose.y) > 5:
            reset_simulation()
	    old_time = rospy.get_time()
	    gazebo_service(EmptyRequest())
	    rospy.sleep(0.5)
        '''

        # check if the robot is falling
        if abs(pitch) > 1:
            reset_simulation()
            old_time = rospy.get_time()
            gazebo_service(EmptyRequest())
            rospy.sleep(0.05)

        K = rospy.get_param('control')
        Km = rospy.get_param('moving')
        delta_time = (rospy.get_time() - old_time)
        old_time = rospy.get_time()

        # moving control 
        em = rospy.get_param("pos") - odom
        em_dot = (em - em_old)/delta_time
        em_old = em
        em_int += em * delta_time
        reference = em * Km['p'] + em_int * Km['i'] + em_dot * Km['d']
        if reference > 0.1: 
            reference = 0.1
        elif reference < -0.1:
            reference = -0.1

        # balancing control 
        e = 0 - pitch
        e_dot = (e - e_old)/delta_time
        e_old = e
        e_int += e * delta_time
        control = e * K['p'] + e_int * K['i'] + e_dot * K['d']

        ctrl_pub.linear.x = -control
        error_pub.publish(Float64(e))
        pub.publish(ctrl_pub)
        rospy.sleep(0.001)

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
