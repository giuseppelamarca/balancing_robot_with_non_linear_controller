#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetModelState,GetModelStateRequest
import numpy as np
from numpy import cos, sin
from nav_msgs.msg import Odometry

#robot params
mb = 0.3
l = 0.12
Ib = 0.002
g = -9.81

pitch = 0
e_old = 0
e_int = 0
em_old = 0
em_int = 0
old_time = 0

ctrl_pub = Twist()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.set_param("reset_simulation",0)

def reset_simulation():
    rospy.loginfo("simulation resetted")
    global e_int, e_old, em_int, em_old, old_time
    e_int = 0
    e_old = 0
    em_old = 0
    em_int = 0
    rospy.wait_for_service("/gazebo/reset_world")
    rospy.wait_for_service("/gazebo/reset_simulation")
    rospy.wait_for_service("/gazebo/get_model_state")
    gazebo_service = rospy.ServiceProxy("/gazebo/reset_world", Empty)
    rospy.ServiceProxy("/gazebo/reset_simulation", Empty)
    robot_pose = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    rospy.set_param("reset_simulation",0)
    rospy.sleep(0.05)


def callback(rpy):
    global pitch
    pitch = rpy.y    

def odom_callback(o):
    global odom
    odom = o.pose.pose.position.x

def swing_up():
    ctrl_pub.linear.x =  0 
    pub.publish(ctrl_pub)
    rospy.sleep(1.5)
    if pitch > 0:
        u = -1.2
    else:
        u = 1.2
    ctrl_pub.linear.x =  u 
    pub.publish(ctrl_pub)
    rospy.sleep(2)
    u = -u
    ctrl_pub.linear.x =  u 
    pub.publish(ctrl_pub)

def controller():
    global e_int, e_old, em_int, em_old, old_time
    rospy.init_node('pid_controller', anonymous=True)

    error_pub = rospy.Publisher("/error_angle", Float64, queue_size=1)

    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.Subscriber("rpy", Vector3, callback)

    reset_simulation()

    old_time = rospy.get_time()
    odom_old = 0
    x_d_old = 0 
    pitch_old = 0
    d_pitch_old = 0
    r = 0     #reference signal in non linear control
    rospy.sleep(3)
    rospy.loginfo("start control loop")
    reset_world = False
    swing_up()
    pitch_old = pitch
    d_pitch = 0
    swing_up()
    control = -1.2
    while(pitch<-0.4):
        pub.publish(ctrl_pub)
    old_time = rospy.get_time()
    while not rospy.is_shutdown():
        K = rospy.get_param('control')
        Km = rospy.get_param('moving')
        delta_time = (rospy.get_time() - old_time)
	if delta_time==0:
		delta_time = 0.001
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
        e = - pitch
        e_dot = (e - e_old)/delta_time
        e_old = e
	if abs(control)<=1.5:
		e_int += e * delta_time
        #control = e * K['p'] + e_int * K['i'] + e_dot * K['d']
        control = e * K['p'] + e_int * K['i'] 
	'''
	if control< -1.2:
		control = -1.2
	elif control > 1.2:
		control = 1.2
	'''
        ctrl_pub.linear.x = -control
        error_pub.publish(Float64(e))
        pub.publish(ctrl_pub)
        rospy.sleep(0.001)

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
