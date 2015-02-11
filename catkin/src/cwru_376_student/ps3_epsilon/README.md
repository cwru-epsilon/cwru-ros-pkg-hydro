Heading
=======

# ps3_epsilon

ps3_epsilon_ is a custom made, 3 part cpp code and a launch file, program which intends to illestrate lidar and estop functionalities of the robot "Jinx".The code was made as to use nodes that publish values on the "cmd_vel" topic.
The nodes are specialized to send to topic robot0/cmd_vel, which works with the STDR simulator.
Change this topic to jinx/cmd_vel to drive the robot "Jinx" in the lab.

The code was divided into three cpp files:
	1. vel_sched_epsilon
	2. lidar_alarm_epsilon
	3. estop_listener_epsilon

The program "vel_sched_epsilon" is a reactive speed scheduler.  It ramps velocity up and down and will recover
from halts, that is for both linear and angular velocities.  To do so, it uses odometry info, published by STDR on topic /robot0/odom.
To run, e.g., on Jinx, change this topic to listen to Jinx's odom messages.

The program "lidar_alarm_epsilon" is where lidar messages are handled. It publishes two topics for other ros nodes to subscribe to. One is "lidar_dist" which is publishing the ping distance (distance to obsticle). The second is "lidar_alarm" which is publishing whether or not the robot is in a safe distance from obsticles (true, false).

The program "estop_listener_epsilon" is handling the hardware estop requests coming from Jinx. Also, it is publishing a topic called "hardware_estop" for other nodes to use.

## Example usage
You can run the program by excuting:
'roslaunch ps3_epsilon ps3_epsilon_launcher.launch'
To run the STDR simulator:  
'roslaunch cwru_376_launchers stdr_glennan_2.launch'
Then run a velocity commander, e.g.:
'rosrun example_robot_commander vel_scheduler'
Can also observe the speed commands by plotting using:
rqt_plot /robot0/cmd_vel/linear/x


    
