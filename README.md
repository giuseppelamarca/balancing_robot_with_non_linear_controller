# non linear control self balancing


#PID controller with starting from the instable equilibrium
roslaunch balancing test.launch

#spawn the robot from the instable equilibrium
roslaunch balancing balancing_gazebo.launch

#spawn the robot from the ground and execute a swing_up (to be improved)
roslaunch balancing swing_up.launch
