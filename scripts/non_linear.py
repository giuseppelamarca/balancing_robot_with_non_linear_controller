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
l = 0.07
Ib = 0.002
g = -9.81

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
    global e_int, e_old, em_int, em_old, old_time, pitch, d_pitch
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
    odom_old = 0
    x_d_old = 0 
    pitch_old = 0
    d_pitch_old = 0
    r = 0     #reference signal in non linear control
    rospy.sleep(1)
    u = 0
    while not rospy.is_shutdown():
	
        K = rospy.get_param('control')
        Km = rospy.get_param('moving')
        delta_time = (rospy.get_time() - old_time)
        old_time = rospy.get_time()

        d_pitch = (pitch - pitch_old)/delta_time
        dd_pitch = (d_pitch - d_pitch_old)/delta_time
        pitch_old = pitch
        d_pitch_old = d_pitch

        # velocity and acceleration base
        x_d = (odom - odom_old)/delta_time
        x_dd= (x_d - x_d_old)/delta_time
        odom_old = odom
        x_d_old = x_d

        #non linear control 
        r = r -0.05
        if r < 0:
            r = 0
        #u = -(mb*l*l + Ib)*dd_pitch - (mb*l)*(cos(pitch)*x_dd - sin(pitch - r) * g )
	M = 0.45
	mb = 0.45
	M_barra = 0.47
	g = -9.81
	l = 0.045
	mu = 0.8	
	Ib =0.0014
	#try to go up 
	#Kp = 158  
	#Kd = 158
	Kp = 168
	Kd = 168
	r =0.0325
	Mg = M * g
	mbl = mb * l
	bw = 0.6
	

	#ud = (-Mg*r*mu - mbl/M_barra * ( Mg* mu - 2 *bw/(r*r) * d_pitch * 0) - (mbl * mbl / M_barra + mbl * l - Ib)*dd_pitch + 1 / (mbl * g) *(-d_pitch - K * pitch)) / (M * r * (1 + r))
	'''
	A = (M * g * sin(pitch) - 0.25 * 0.07 * cos(pitch) * sin(pitch) * d_pitch * d_pitch) 
	B = (0.25 * 0.07 * cos(pitch) * cos(pitch) - M * 0.07) 
	C = (-Kp * pitch - Kd * d_pitch)
	D = M*g*mu*cos(pitch)
	ud = (( A + B * C )/cos(pitch) - D) / 20#( M * r )
	u = u + ud * delta_time
	'''
	M = 0.02
	m = 0.45
	Kp = 140#138 #220
	Kd = 180#138 #220
	#pitch = 0
	#d_pitch = 0
	A = ((M+m) * g * sin(pitch) - m*l * cos(pitch) * sin(pitch) * d_pitch * d_pitch) 
	B = (m * l * cos(pitch) * cos(pitch) - (M + m)*l) 
	C = (-Kp * pitch - Kd * d_pitch)
	D = (M+m)*g*mu*sin(pitch)
	E = (( A + B * C )/cos(pitch) - D)

	if E > 0.1:
		E = 0.1
	elif E < -0.1:
		E = -0.1
	
	ud = E /( (M + m) * r )
	u = u + ud * delta_time
	#print("{}	{}	{}	{}".format(A,B,C,D))
	print("{}".format(E))

	if pitch >0:
		u = 0.8
	else:
		u = -0.8
        ctrl_pub.linear.x = u 
        pub.publish(ctrl_pub)
        rospy.sleep(0.001)

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
