
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

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message time
#include <string>


const double MIN_SAFE_DISTANCE = 0.6; // set alarm if anything is within 0.5m of the front of robot
const std::string lidarT = "base_laser1_scan"; // /robot0/laser_0 or "base_laser1_scan" or /laser/scan


// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_= 0; // NOT real; callback will have to find this
int ping_index_min_ = 40;
int ping_index_max_ = 74;
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    for (ping_index_ = ping_index_min_; ping_index_ <= ping_index_max_; ping_index_= ping_index_ + 5) {
        ping_dist_in_front_ = laser_scan.ranges[ping_index_];
        ROS_INFO("ping dist in front = %f",ping_dist_in_front_);
        if (ping_dist_in_front_<MIN_SAFE_DISTANCE) {
            ROS_WARN("DANGER, WILL ROBINSON!!");
            laser_alarm_=true;
            
        }
        else if (laser_alarm_) break;
        else {
            laser_alarm_=false;
        }
        std_msgs::Float32 lidar_dist_msg;
        lidar_dist_msg.data = ping_dist_in_front_;
        lidar_dist_publisher_.publish(lidar_dist_msg);  
    }
	laser_alarm_ = false;
    std_msgs::Bool lidar_alarm_msg;
    lidar_alarm_msg.data = laser_alarm_;
    lidar_alarm_publisher_.publish(lidar_alarm_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm_epsilon_v2"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe(lidarT, 1, laserCallback); // robot0/laser_0 for simulation robot.   
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}

