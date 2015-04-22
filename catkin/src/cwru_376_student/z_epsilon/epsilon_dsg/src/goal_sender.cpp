//epsilon path sender
// aab119, Mar 2015
// test node compatible with epsilon_des_state_generator.cpp;
// transmits a hard-coded path to desired-state generator node via service "appendPathService"
// the message must contain a nav_msgs/Path object

#include <stdlib.h>
#include <string>
#include <vector>
#include <queue>
#include <iostream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>

#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include <cwru_srv/path_service_message.h>

//utility to convert from heading to quaternion, planar motion
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi/2.0);
    quaternion.w = cos(phi/2.0);
    return(quaternion);
}

//utility to fill a Pose object from planar x,y,phi info
geometry_msgs::Pose xyPhi2Pose(double x, double y, double phi) {
    geometry_msgs::Pose pose; // a pose object to populate
    pose.orientation = convertPlanarPhi2Quaternion(phi); // convert from heading to corresponding quaternion
    pose.position.x = x; // keep the robot on the ground!
    pose.position.y = y; // keep the robot on the ground!    
    pose.position.z = 0.0; // keep the robot on the ground!  
    return pose;
}

int main(int argc, char **argv) {
    double dt=0.01;
    ros::init(argc, argv, "path_sender"); // name of this node 
    ros::NodeHandle nh; 
    ros::ServiceClient client = nh.serviceClient<cwru_srv::path_service_message>("appendPathService");

    cwru_srv::path_service_message path_message;
    geometry_msgs::PoseStamped vertex;  // this contains a header and a pose; the pose contains a point and a quaternion
    double x,y,phi;

    vertex.header.stamp = ros::Time::now(); // look up the time and put it in the header; use same time stamp for all vertices in this path
    vertex.header.frame_id = "map"; // specify this, so tf will know how to transform it
    
    // fill in the interesting data: (x,y) and phi = location and heading THOSE ARE FROM ACTUAL AMCL MAP POINTS.....

    //Goal Pose (Close to the table):
    x=5.14055403825;
    y=12.0505654959;
    phi=-0.924655161966;

    ROS_INFO("vertex: x,y,phi = %f, %f %f",x,y,phi);
    vertex.pose = xyPhi2Pose(x,y,phi); //x,y,phi
    path_message.request.path.poses.push_back(vertex);

    //Home Pose (Home): 
    //x=-3.31683585831;
    //y=20.6154034727;
    //phi=0.926395175302;

    //ROS_INFO("vertex: x,y,phi = %f, %f %f",x,y,phi);
    //vertex.pose = xyPhi2Pose(x,y,phi); //x,y,phi
    //path_message.request.path.poses.push_back(vertex);
    

    //transmit this path message:
    if (client.call(path_message)) {
        ROS_INFO("got ack from server");
        } else {
            ROS_ERROR("Failed to call service lookup_by_name");
            return 1;
        }
    return 0;
}
