// process_pcd_save.cpp: wsn, April, 2015
// example code to acquire a pointcloud from a topic, and save a snapshot to disk
// as a PCD file.

#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> 
#include <geometry_msgs/PointStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/features/normal_3d.h>



#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
//#include <pcl/common/impl/centroid.hpp>

//PointCloud::Ptr pclCloud(new PointCloud); // Holds the whole pcl cloud
//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

#include <tf/transform_listener.h> //for transforms

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace pcl::io;

pcl::PointCloud<pcl::PointXYZ>::Ptr g_pclKinect(new PointCloud<pcl::PointXYZ>);

bool got_cloud = false;
bool saved_file = false;

const tf::TransformListener* tfListener_;
tf::StampedTransform kToB_;    

void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclKinect(new PointCloud<pcl::PointXYZ>);
    //const sensor_msgs::PointCloud2ConstPtr temp = kinect_to_base_link(*cloud);
    pcl::fromROSMsg(*cloud, *g_pclKinect);
    //ROS_INFO("kinectCB %d * %d points", pclKinect->width, pclKinect->height);
        got_cloud=true; //cue to "main" to save snapshot
}

//sensor_msgs::PointCloud2 kinect_to_base_link(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    // to use tf, need to convert coords from a geometry_msgs::Pose into a tf::Point

  // sensor_msgs::PointCloud2 temp_pcl; 
    
    //tfListener_->lookupTransform("base_link", "kinect_pc_frame", ros::Time(0), kToB_);
    //let's transform the map_pose goal point into the odom frame:
    //cloud->header.stamp = ros::Time::now();
    //tfListener_->transformPointCloud ("base_link", cloud, temp_pcl);

    //return temp_pcl; // 
//}

int main(int argc, char** argv) {
    // Do some initialization here
    ros::init(argc, argv, "process_pcl");
    ros::NodeHandle nh;
    ros::Rate rate(2);

    bool tferr=true;
    ROS_INFO("waiting for tf between kinect_pc_fram and base_link...");
//    while (tferr) {
//        tferr=false;
//        try {
                //try to lookup transform from target frame "base_link" to source frame "kinect_pc_fram"
            //The direction of the transform returned will be from the target_frame to the source_frame. 
             //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
//                tfListener_->lookupTransform("base_link", "kinect_pc_frame", ros::Time(0), kToB_);
//            } catch(tf::TransformException &exception) {
//                ROS_ERROR("%s", exception.what());
//                tferr=true;
//                ros::Duration(0.5).sleep(); // sleep for half a second
//                ros::spinOnce();                
//            }   
//    }
//    ROS_INFO("tf is good");
    // Subscribers
    ros::Subscriber getPCLPoints = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect/depth/points", 1, kinectCB);

    
    
    while (ros::ok()) {       
        if (got_cloud) {
            if (!saved_file) {
                pcl::io::savePCDFileASCII ("test_pcd.pcd", *g_pclKinect);
                std::cerr << "Saved " << g_pclKinect->points.size () << " data points to test_pcd.pcd." << std::endl;
                std::cout<<"frame: "<<g_pclKinect->header.frame_id<<std::endl;
                saved_file=true; // mark that we have saved a file--don't need to repeat this; just do it once                
                return 0; // or bail out now
            }
        }
        else 
            cout<<"waiting for got_cloud..."<<endl;
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
