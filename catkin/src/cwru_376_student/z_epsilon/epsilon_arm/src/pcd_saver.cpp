// process_pcd_save.cpp: wsn, April, 2015
// example code to acquire a pointcloud from a topic, and save a snapshot to disk
// as a PCD file.

// For Transformation of points to base_link, I followed the example in the below link.
//http://lars.mec.ua.pt/lartk/doc/plane_model_road_segmentation/html/transform__cloud__nodelet_8cpp_source.html


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

#include <pcl/filters/extract_indices.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
//#include <pcl/common/impl/centroid.hpp>

//PointCloud::Ptr pclCloud(new PointCloud); // Holds the whole pcl cloud
//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
#include <pcl_ros/transforms.h>
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
    //ROS_WARN("Original Cloud Frame ID: %f", cloud->header.frame_id );
    pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_trans;
    //sensor_msgs::PointCloud2 cloud_ok;
    
    //cloud_ok = *cloud;
    //STEP 1 Convert sensor_msgs to pcl
    pcl::fromROSMsg(*cloud,cloud_in);
    //const ros::Time temp = ros::Time::now();
    //cloud->header.stamp=ros::Time::now();
    //*cloud = cloud_ok;
    //STEP 2 Convert xb3 message to center_bumper frame (i think it is better this way)
    try {
        //cloud->header.stamp=ros::Time::now();
        tfListener_->lookupTransform("base_link", "camera_depth_optical_frame", cloud->header.stamp, kToB_);
    }
    catch (tf::TransformException ex)  {
        ROS_ERROR("%s",ex.what());
    }
    // Transform point cloud
    pcl_ros::transformPointCloud (cloud_in,cloud_trans,kToB_);  
    cloud_trans.header.frame_id="base_link";

    cloud_trans.header.stamp= ros::Time::now().toSec();
    //For us to fetch cloud_trans into g_pclKinect, Here is a trick... :)
    sensor_msgs::PointCloud2 temp;  
    pcl::toROSMsg(cloud_trans, temp);//*g_pclKinect);
    pcl::fromROSMsg(temp, *g_pclKinect);
    //ROS_INFO("kinectCB %d * %d points", pclKinect->width, pclKinect->height);
    got_cloud=true; //cue to "main" to save snapshot
}

int main(int argc, char** argv) {
    // Do some initialization here
    ros::init(argc, argv, "process_pcl");
    ros::NodeHandle nh;
    ros::Rate rate(2);
    
    ROS_INFO("waiting for tf between kinect_pc_fram and base_link...");
    tf::TransformListener listener_;
    tfListener_=&listener_;
    ROS_INFO("tf is good");
    // Subscribers
    ros::Subscriber getPCLPoints = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, kinectCB);
    
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
