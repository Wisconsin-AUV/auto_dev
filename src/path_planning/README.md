Custom OdometrySimple message is used for the /wauv/goal_odometry msg

-Subscribes to "/mavros/local_position/odom" in order to get the current robot odometry
-Subscribes to "/wauv/goal_odometry" in order to get the goal odometry for the robot to go to
-Publishes to "/mavros/setpoint_velocity/cmd_vel" to give the command velocities for the robot to follow

-PID parameters can be changed using "ros2 param set /path_planning <parameter> <value>"
where value is a double (e.g. 0.0) and parameter can be any of the following:
-position_kp, position_ki, position_kd
-orientation_kp, orientation_ki, orientation_kd
-trajectory_kp, trajectory_ki, trajectory_kd
