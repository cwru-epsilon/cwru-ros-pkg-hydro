
// The original code credit goes to: cwru-robotics team
// https://github.com/cwru-robotics/
// Dr. Wyatt Newman
// Engr. Luc Battaieb

// This project is one of EECS378 Mobile Robotics assignments, Spring 2015.

// This project was edited in order to modulate the velocity (linear and angular) commands 
// to comply with a speed limit, v_max and omega_max, acceleration limits, +/-a_max and +/-alpha_max,
// and come to a halt gracefully at the end of each intended segment...

// Project Team: (Team Epsilon ε)
// - Alaa Badokhon
// - Josh Immerman
// - Dongyu Wu
// - Eric Carlson
// https://github.com/cwru-epsilon

// try this, e.g. with roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch
// or: roslaunch cwru_376_launchers stdr_glennan_2.launch
// watch resulting velocity commands with: rqt_plot /jinx/cmd_vel/linear/x (or jinx/cmd_vel...)

// notes on quaternions:
/*
From:
http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/
qx = ax * sin(angle/2)
qy = ay * sin(angle/2)
qz = az * sin(angle/2)
qw = cos(angle/2)
so, quaternion in 2-D plane (x,y,theta):
ax=0, ay=0, az = 1.0
qx = 0;
qy = 0;
qz = sin(angle/2)
qw = cos(angle/2)
therefore, theta = 2*atan2(qz,qw)
*/
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <std_msgs/Bool.h>
#include <string>
#include <cwru_srv/simple_bool_service_message.h>
#include <cwru_srv/simple_int_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include <cwru_srv/path_service_message.h>

// set some dynamic limits...
const double v_max = 0.6; //1m/sec is a fast walk (we decided to make it 0.6 for actual demo on Jinx)
//const double v_min = 0.1; // if command velocity too low, robot won't move
const double a_max = 1.6; //1m/sec^2 is 0.1 g's (This value is just enough to command Jinx to move on top of the treadmill)

const double omega_max = 2.0; //1 rad/sec-> about 6 seconds to rotate 1 full rev 
const double alpha_max = 3.0; //0.5 rad/sec^2-> takes 2 sec to get from rest to full omega 
const double DT = 0.050; // choose an update rate of 20Hz; go faster with actual hardware

const std::string odomT = "/odom"; // /robot0/odom or /odom
const std::string cmd_velT = "/cmd_vel"; // robot0/cmd_vel or /cmd_vel


// globals for communication w/ callbacks:
double odom_vel_ = 0.0; // measured/published system speed
double odom_omega_ = 0.0; // measured/published system yaw rate (spin)
double odom_x_ = 0.0; 
double odom_y_ = 0.0;
double odom_phi_ = 0.0;
double dt_odom_ = 0.0;
double odom_twist_z = 0.0; //to adjust robot drift.
ros::Time t_last_callback_;
double dt_callback_=0.00;

const double MIN_SAFE_DISTANCE = 0.55; //in meters for Lidar
bool pause_soft = false;
bool pause_hard = false;
bool pause_lidar = false;
bool print_soft = true;
bool print_hard = true;
bool print_lidar = true;
bool print_all = true;
double rem_dist_ = 0.0;
bool rot_value;

bool spinCheck = true;
bool incOdomPhi = false;
double odom_phi_old = 0.0;
double phiDiff;

bool move_back = false;

// receive odom messages and strip off the components we want to use
// tested this OK w/ stdr
// receive the pose and velocity estimates from the simulator (or the physical robot)
// copy the relevant values to global variables, for use by "main"
// Note: stdr updates odom only at 10Hz; Jinx is 50Hz (?)

double speedCompare (double odom_speed, double sched_speed, bool rotate ) {
    odom_speed = sqrt(odom_speed * odom_speed); // To make it always +ve
    sched_speed = sqrt(sched_speed * sched_speed); // To make it always +ve
    double accel = a_max;
    if (rotate) {
        accel = alpha_max;
    }
    //how does the current velocity compare to the scheduled vel?
    if (odom_speed < sched_speed) { // maybe we halted, e.g. due to estop or obstacle;
    // may need to ramp up to v_max; do so within accel limits
        double v_test = odom_speed + accel*dt_callback_; // if callbacks are slow, this could be abrupt
        
        return (v_test < sched_speed) ? v_test : sched_speed; //choose lesser of two options
        
    // this prevents overshooting scheduled_vel
    } else if (odom_speed > sched_speed) { //travelling too fast--this could be trouble
    // ramp down to the scheduled velocity. However, scheduled velocity might already be ramping down at a_max.
    // need to catch up, so ramp down even faster than a_max. Try 1.2*a_max.
         if (rotate && print_all) ROS_INFO("odom omega: %f; sched omega: %f", odom_speed, sched_speed); //debug/analysis output; can comment this out
         else if (print_all) ROS_INFO("odom velocity: %f; sched velocity: %f", odom_speed, sched_speed);
        double v_test = odom_speed - 1.2 * accel*dt_callback_; //1.2 * accel*dt_callback_; //moving too fast--try decelerating faster than nominal a_max
        return (v_test > sched_speed) ? (3*v_test+sched_speed)/4  : sched_speed;// sched_speed : v_test ; // choose larger of two options...don't overshoot scheduled_vel
        
    } else {
        return sched_speed; //silly third case: this is already true, if here. Issue the scheduled velocity
    }
} 


// The masterLoop method handles the node from main() and takes either a length forward or phi to rotate to. 
// A boolean was defined to distinguish between the two...
double masterLoop(ros::NodeHandle& nh, double seg_len, bool rotate, double rot_phi) {
    
//---------------------------Initialization--------------------------------------------
// This initialization happen only once in every function call in main().    
    
// Create a publisher object that can talk to ROS and issue twist messages on named topic;
// Note: this is customized for stdr robot; would need to change the topic to talk to jinx, etc.
    // The name of the topic in Jinx is /cmd_vel
    ros::Publisher vel_cmd_publisher = nh.advertise<geometry_msgs::Twist>(cmd_velT, 1); 
    ros::Rate rtimer(1 / DT); // frequency corresponding to chosen sample period DT; the main loop will run this fast
// desired travel distance in meters; anticipate travelling multiple segments
// here's a subtlety: might be tempted to measure distance to the goal, instead of distance from the start.
// HOWEVER, will NEVER satisfy distance to goal = 0 precisely, but WILL eventually move far enough to satisfy distance travelled condition
    
    double segment_length_done = 0.0; // need to compute actual distance travelled within the current segment
    double start_x = 0.0; // fill these in with actual values once Odom message is received
    double start_y = 0.0; // subsequent segment start coordinates should be specified relative to end of previous segment
    double scheduled_vel = 0.0; //desired vel, assuming all is per plan
    double new_cmd_vel = 0.1; // value of speed to be commanded; update each iteration
    
    double start_phi = 0.0; // Compare with odom_phi
    double rotation_done = 0.0; // need to compute actual rotation done in Radian
    double new_cmd_omega = 0.0; // update spin rate command as well
    double scheduled_omega = 0.0; //desired vel, assuming all is per plan
    
    geometry_msgs::Twist cmd_vel; //create a variable of type "Twist" to publish speed/spin commands
    cmd_vel.linear.x = 0.0; // initialize these values to zero
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
    
// let's wait for odom callback to start getting good values..
    
    odom_omega_ = 1000000; // absurdly high
    ROS_INFO("waiting for valid odom callback...");
    t_last_callback_ = ros::Time::now(); // initialize reference for computed update rate of callback
    while (odom_omega_ > 1000) {
        rtimer.sleep();
        ros::spinOnce();
    }
    ROS_INFO("received odom message; proceeding");
    start_x = odom_x_;
    start_y = odom_y_;
    start_phi = odom_phi_;
    ROS_INFO("start pose: x %f, y= %f, phi = %f", start_x, start_y, start_phi);
    
// compute some properties of trapezoidal velocity profile plan:
    
    double T_accel = v_max / a_max; //...assumes start from rest
    double T_decel = v_max / a_max; //(for same decel as accel); assumes brake to full halt
    double dist_accel = 0.5 * a_max * (T_accel * T_accel); //distance rqd to ramp up to full speed
    double dist_decel = 0.5 * a_max * (T_decel * T_decel); //same as ramp-up distance
    double R_accel = omega_max / alpha_max;
    double R_decel = omega_max / alpha_max;
    
    double spinIncrement = sqrt((start_phi+rot_phi)*(start_phi+rot_phi)); //This is the value of the spin requested after the travel... 
    //double R_dist_accel = 0.5 * alpha_max * (R_accel * R_accel);
    //double R_dist_decel = 0.5 * alpha_max * (R_decel * R_decel);
    
//-----------------------------------------------------------------------------------
//----------------------How to Pause The Robot---------------------------------------
    
    //rostopic pub [topic] [msg_type] [args]
    // In Terminal...
    //rostopic pub soft_estop std_msgs/Bool 'true' // To toggle the soft Estop from Terminal
   
    
//-----------------------------------------------------------------------------------    
// Do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted (or ctl-C)
// The following loop is for forward/backward motion    
    while (ros::ok() && rotate == false )
       {
        ros::spinOnce(); // allow callbacks to populate fresh data
        
    // compute distance travelled so far:
        double delta_x = odom_x_ - start_x;
        double delta_y = odom_y_ - start_y;
        segment_length_done = sqrt(delta_x * delta_x + delta_y * delta_y);
        ROS_INFO("dist travelled: %f", segment_length_done);
        double dist_to_go = seg_len - segment_length_done;
        
//----------------------------------------------------------------------------------
        
    //use segment_length_done to decide what vel should be, as per plan
        if (dist_to_go<= 0.0) { // at goal, or overshot; stop!
            scheduled_vel=0.0;
        }
        else if (dist_to_go <= dist_decel) { //possibly should be braking to a halt
            scheduled_vel = sqrt(2 * dist_to_go * a_max);
            ROS_WARN("Breaking ZOne: %f", scheduled_vel);
        }
        else { // not ready to decel, so target vel is v_max, either accel to it or hold it
            scheduled_vel = v_max;
        }
        
        ROS_INFO("sched vel:--- %f", scheduled_vel);
//------------------------------------------------------------------------------------
        // Compare Odom velocity with requested velocity and return the appropriate value
        new_cmd_vel = speedCompare(odom_vel_, scheduled_vel, false); 
//------------------------------------------------------------------------------------
        ROS_INFO("cmd vel: %f",new_cmd_vel); // debug output
        cmd_vel.linear.x = new_cmd_vel;
        if (dist_to_go <= 0.0) { //uh-oh...went too far already!
            cmd_vel.linear.x = 0.0; //command vel=0
        }
        // If there was any trigger, stop until  that trigger is released...
        while (pause_soft || pause_hard || pause_lidar) {
            print_all = false;
            if (pause_soft && print_soft) { 
                ROS_WARN("Stopping because of Soft Estop");
                print_soft = false;
            }
            if (pause_hard && print_hard) {
                ROS_WARN("Stopping because of Hard Estop");
                print_hard = false;
            }
            if (pause_lidar && print_lidar) {
                ROS_WARN("Stopping because an obstacle was detected by the Lidar");
                print_lidar = false;
            }
            //new_cmd_vel = speedCompare(odom_vel_, 0.0, false); 
            cmd_vel.linear.x = 0.0; // 
            cmd_vel.angular.z = 0.0;
            vel_cmd_publisher.publish(cmd_vel);
            if (dist_to_go <= seg_len*0.15) //So for 5.5 = 0.825 m So if less than that just return and get out of the function.
                return segment_length_done;
            ros::spinOnce();
        }
//        if (exit) {
//            exit = false;
//            break;
//        }
        print_soft = true;
        print_hard = true;
        print_lidar = true;
        cmd_vel.angular.z = -odom_twist_z*7; // to adjust Abby's drift..$
        vel_cmd_publisher.publish(cmd_vel); // publish the command to /cmd_vel in Jinx 
        
        rtimer.sleep(); // sleep for remainder of timed iteration
        if (dist_to_go <= 0.0) break; // halt this node when this segment is complete.
        rem_dist_ = dist_to_go;
        rot_value = rotate;
    }
     
    while (ros::ok() && rotate == true) // WHEN THERE IS ROTATION $$$$$$$$$$$$$$$$
    {
        ros::spinOnce(); // Allow Callbacks to populate fresh data
//------------------------Odom Phi Checker and Fixer------------------------------
        if (!spinCheck) {
            phiDiff = abs(odom_phi_ - odom_phi_old);
            if (phiDiff >= 1) { //The moment when it begins a new circle...
                incOdomPhi = true;
                spinCheck = true; // Just to not go through this if statement again after this point...
                spinIncrement = 0.0; // Just to not do the following else if statement and proceed with the final else if
            }
        }
        if (spinIncrement>6.28) {
            odom_phi_old = odom_phi_; // To save the previous Odom Phi value (for making sure that it stays in the same travel phi...)
            spinCheck = false;
        }
        else if (incOdomPhi) {
            odom_phi_ = odom_phi_ + odom_phi_old;
        }
        ROS_INFO("Modified (checked) Odom_phi = %f", odom_phi_);
//-----------------------------------------------------------------------------------        
        rotation_done = odom_phi_ - start_phi;
        ROS_INFO("Rotation Traveled: %f", rotation_done);
        double rot_to_go = rot_phi - rotation_done;
        double percent_left = rot_to_go/rot_phi * 100;
        ROS_INFO("Rotation ToDo: %f, percent_left = %f", rot_to_go, percent_left);
//------------------------------------------------------------------------------------
//use segment_length_done to decide what OMEGA should be, as per plan
        if (floor(rot_to_go*100)/100 == 0.0 || percent_left < 1.0) { // at goal, or overshot; stop!
            scheduled_omega=0.0;
        }
        else if (percent_left >= 20 || percent_left <=80) { //possibly should be braking to a halt // floor(sqrt(rot_to_go*rot_to_go)*10)/10 <= floor(R_dist_decel/10)*10
            scheduled_omega = sqrt(2 * sqrt(rot_to_go*rot_to_go) * alpha_max);
            ROS_INFO("Rotation braking zone: omega_sched = %f",scheduled_omega);
        }
        else { // not ready to decel, so target vel is v_max, either accel to it or hold it
            scheduled_omega = omega_max;
        }
//------------------------------------------------------------------------------------
        new_cmd_omega = speedCompare(odom_omega_, scheduled_omega, true);
//------------------------------------------------------------------------------------
        // Make the speed of rotation right NOW as new_cmd_OMEGA with the right direction of PHI (+ve or -ve).
        cmd_vel.angular.z = (rot_to_go < 0.0) ? (-1)*new_cmd_omega : new_cmd_omega;
        
        if (floor(rot_to_go*100)/100 == 0.0 || percent_left < 1.0) { //uh-oh...went too far already!
             cmd_vel.angular.z = 0.0; //command omega=0
        }
        // If there was any trigger, stop until  that trigger is released...
        while (pause_soft || pause_hard || pause_lidar) {
            print_all = false;
            if (pause_soft && print_soft) { 
                ROS_WARN("Stopping because of Soft Estop");
                print_soft = false;
            }
            if (pause_hard && print_hard) {
                ROS_WARN("Stopping because of Hard Estop");
                print_hard = false;
            }
            if (pause_lidar && print_lidar) {
                ROS_WARN("Stopping because an obstacle was detected by the Lidar");
                print_lidar = false;
            }
            //new_cmd_vel = speedCompare(odom_vel_, 0.0, false); 
            cmd_vel.linear.x = 0.0; // 
            cmd_vel.angular.z = 0.0;
            vel_cmd_publisher.publish(cmd_vel);
            ros::spinOnce();
        }
        print_soft = true;
        print_hard = true;
        print_lidar = true;
        vel_cmd_publisher.publish(cmd_vel); // publish the command to jinx/cmd_omega
        rtimer.sleep(); // sleep for remainder of timed iteration
        if (floor(rot_to_go*100)/100 == 0.0 || percent_left < 1.0) break; // halt this node when this segment is complete.    
    }
}
// This Callback is for handling hard estop triggers (Hardware estops are handled in estop_listener_epsilon.cpp ..
void hardEstopCallback (const std_msgs::Bool& estop_hard) {
    if (estop_hard.data == true) {
        if (print_hard) ROS_WARN("Hard Estop was triggered and ON (Robot is stopped)");
        pause_hard = true;
    }
    else if (estop_hard.data == false && pause_hard == true) {
        ROS_WARN("Hard Estop was triggered and OFF (Robot is in motion) :)");
        pause_hard = false;
    }
}

// This Callback is for handling Lidar detections, handled in lidar_alarm_epsilon.cpp..
void laserMsgCallback (const std_msgs::Float32& dist) {
    //ROS_INFO("Lidar: distance to obstacle is %f", dist.data);
    if (dist.data<MIN_SAFE_DISTANCE) {
        if (print_lidar) ROS_WARN("DANGER, WILL ROBINSON!!, Obstacle in %f meters... ", dist.data);
        pause_lidar = true;
    }
    else if (dist.data>=MIN_SAFE_DISTANCE && pause_lidar) {
		pause_lidar = false; //if (dist.data>MIN_SAFE_DISTANCE && pause_lidar) pause_lidar = false;
		ROS_WARN("Obstacle is out of the way now... :)");
	}    
}

// This Callback is for handling hard estop triggers...
void softEstopCallback (const std_msgs::Bool& estop_soft) {
    if (estop_soft.data == true) {
        if (print_soft) ROS_WARN("Soft Estop was triggered and ON");
        pause_soft = true;
    }
    else if (estop_soft.data == false && pause_soft == true) {
        ROS_WARN("Soft Estop was triggered and OFF :)");
        pause_soft = false;
        //if (rot_value) masterLoop(nh, 0.0, true, rem_dist_);
        //else masterLoop(nh, rem_dist_, false, 0.0 );
    }
 }

// This Callback is for handling and updating Odom values...
void odomCallback(const nav_msgs::Odometry& odom_rcvd) {
//here's a trick to compute the delta-time between successive callbacks:
    dt_callback_ = (ros::Time::now() - t_last_callback_).toSec();
    t_last_callback_ = ros::Time::now(); // let's remember the current time, and use it next iteration
    if (dt_callback_ > 0.15) { // on start-up, and with occasional hiccups, this delta-time can be unexpectedly large
        dt_callback_ = 0.1; // can choose to clamp a max value on this, if dt_callback is used for computations elsewhere
        ROS_WARN("large dt; dt = %lf", dt_callback_); // let's complain whenever this happens
    }
    //ROS_WARN("dt_callback ==== %f", dt_callback_);
// copy some of the components of the received message into global vars, for use by "main()"
// we care about speed and spin, as well as position estimates x,y and heading
    odom_vel_ = odom_rcvd.twist.twist.linear.x;
    odom_omega_ = odom_rcvd.twist.twist.angular.z;
    odom_x_ = odom_rcvd.pose.pose.position.x;
    odom_y_ = odom_rcvd.pose.pose.position.y;
//odom publishes orientation as a quaternion. Convert this to a simple heading
// see notes above for conversion for simple planar motion
    double quat_z = odom_rcvd.pose.pose.orientation.z;
    double quat_w = odom_rcvd.pose.pose.orientation.w;
    odom_phi_ = 2.0*atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
// the output below could get annoying; may comment this out, but useful initially for debugging
    //ROS_INFO("odom CB: x = %f, y= %f, phi = %f, v = %f, omega = %f", odom_x_, odom_y_, odom_phi_, odom_vel_, odom_omega_);
	odom_twist_z = odom_rcvd.twist.twist.angular.z;
}

bool moveService(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response)
{
    ROS_INFO("Move service callback activated");
    response.resp = true; // boring, but valid response info
    move_back=true; //inform "main" that we have a new goal!
    return true;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "vel_sched_epsilon_v2"); // name of this node will be "vel_sched_epsilon"
    ros::NodeHandle nh; // get a ros nodehandle; standard yadda-yadda
    ros::Subscriber vel_sub = nh.subscribe(odomT, 1, odomCallback); // Subscribing to jinx/odom (which has to be changed to jinx/odom for actual robot simulation).
    ros::Subscriber lidar_msg_sub = nh.subscribe("lidar_dist", 1, laserMsgCallback); // Subscribing to lider_dist, which is published or advertised by lidar_alarm_epsilon.cpp
    ros::Subscriber soft_estop = nh.subscribe("soft_estop", 1, softEstopCallback); // Subscribing to soft_estop, which is published by the user (manually on the terminal)
    ros::Subscriber hard_estop = nh.subscribe("hardware_estop", 1, hardEstopCallback); // Subscribing to hardware_estop, which is published by estop_listener_epsilon.cpp
    ros::ServiceServer service = nh.advertiseService("move_back", moveService); 
// here is a description of some segments of a journey.
// define the desired path length of this segment and wither or not their was needed a rotation (both moving forward and rotation cannot happen at once)

    double dist_back = masterLoop(nh, 5.5, false, 0.0);
    while(ros::ok()) {// Wait here until you get a trigger from user
        if (move_back) break;
        ros::spinOnce(); // Allow Callbacks to populate fresh data
    }
    ROS_WARN("Going Back HOME :) ");
    masterLoop(nh, 0.0, true, -3.00);
    masterLoop(nh, dist_back-0.5, false, 0.0); //0.5 travveled backwards using leave _table
    //masterLoop(nh, 0.0, true, -0.20);
    //masterLoop(nh, 4.5, false, 0.0);

    ROS_INFO("completed move distance");
}
