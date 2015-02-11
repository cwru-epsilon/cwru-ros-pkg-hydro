
// The original code credit goes to: cwru-robotics team
// https://github.com/cwru-robotics/
// Dr. Wyatt Newman
// Engr. Luc Battaieb

// This project is one of EECS378 Mobile Robotics assignments, Spring 2015.

// This project was edited in order to modulate the velocity (linear and angular) commands to comply with a speed limit, v_max and omega_max,
// acceleration limits, +/-a_max and +/-alpha_max, and come to a halt gracefully at the end of each intended segment...

// Project Team: (Team Epsilon ε)
// - Alaa Badokhon
// - Josh Immerman
// - Dongyu Wu
// - Eric Carlson
// https://github.com/cwru-epsilon

#include <ros/ros.h>
#include <cwru_msgs/Pose.h>
#include <iostream>
#include <fstream>
#include <cwru_msgs/cRIOSensors.h>
#include <std_msgs/Bool.h>
using namespace std;

bool estop;
string check;
ros::Publisher hard_estop_pub;

void estopCallback(const std_msgs::Bool::ConstPtr& estop) 
{
    std_msgs::Bool estop_msg;
    if (estop->data == true) {
        check = "estop_off";  // means motors are ENABLED
        estop_msg.data = false;
        hard_estop_pub.publish(estop_msg);
    }
    else if (estop->data == false) {
        check = "estop_on";  // means motors are DISABLED    
        estop_msg.data = true;
        hard_estop_pub.publish(estop_msg);
    }
    
    cout<<check<<endl;
}

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line. For programmatic
	 * remappings you can use a different version of init() which takes remappings
	 * directly, but for most command-line programs, passing argc and argv is the easiest
	 * way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "estop_listener_epsilon");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;

	/**
	 * The subscribe() call is how you tell ROS that you want to receive messages
	 * on a given topic.  This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing.  Messages are passed to a callback function, here
	 * called chatterCallback.  subscribe() returns a Subscriber object that you
	 * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	 * object go out of scope, this callback will automatically be unsubscribed from
	 * this topic.
	 *
	 * The second parameter to the subscribe() function is the size of the message
	 * queue.  If messages are arriving faster than they are being processed, this
	 * is the number of messages that will be buffered up before beginning to throw
	 * away the oldest ones.
	 */
	ros::Subscriber sub = n.subscribe("motors_enabled",1,estopCallback);
        hard_estop_pub = n.advertise<std_msgs::Bool>("hardware_estop", 1);

	/**
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */
	ros::spin();

	return 0;
}

