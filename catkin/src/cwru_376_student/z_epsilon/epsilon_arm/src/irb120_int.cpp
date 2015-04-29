// simple_marker_listener.cpp
// Wyatt Newman
// node that listens on topic "marker_listener" and prints pose received

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <interactive_markers/interactive_marker_server.h>
#include <irb120_kinematics.h>
#include <cwru_srv/simple_bool_service_message.h>
#include <cwru_srv/simple_int_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include <cwru_srv/path_service_message.h>
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <sensor_msgs/JointState.h>
 #include <tf/transform_listener.h>

//callback to subscribe to marker state
Eigen::Vector3d g_p, g_p_above;
Vectorq6x1 g_q_state;
Vectorq6x1 home_state, trans_state; //= {0.0245981, -1.43767, 0.0514272, -0.13549, -0.363732, -0.0716817};

double g_x,g_y,g_z;
//geometry_msgs::Quaternion g_quat; // global var for quaternion
Eigen::Quaterniond g_quat;
Eigen::Matrix3d g_R, g_R_above;
Eigen::Affine3d g_A_flange_desired, g_A_flange_desired_a;
bool g_trigger=false;

 
tf::TransformListener* g_tfListener;
tf::StampedTransform g_armlink1_wrt_baseLink;
geometry_msgs::PoseStamped g_marker_pose_in;
geometry_msgs::PoseStamped g_marker_pose_wrt_arm_base;

using namespace std;

void markerListenerCB(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    ROS_INFO_STREAM("marker frame_id is "<<feedback->header.frame_id);
    g_marker_pose_in.header = feedback->header;
    g_marker_pose_in.pose=feedback->pose;
     g_tfListener->transformPose("link1", g_marker_pose_in, g_marker_pose_wrt_arm_base);
     
    //copy to global vars:
     /*
    g_p[0] = feedback->pose.position.x;
    g_p[1] = feedback->pose.position.y;
    g_p[2] = feedback->pose.position.z;
    g_quat.x() = feedback->pose.orientation.x;
    g_quat.y() = feedback->pose.orientation.y;
    g_quat.z() = feedback->pose.orientation.z;
    g_quat.w() = feedback->pose.orientation.w;   
    g_R = g_quat.matrix(); */
     
    g_p[0] = g_marker_pose_wrt_arm_base.pose.position.x;
    g_p[1] = g_marker_pose_wrt_arm_base.pose.position.y;
    g_p[2] = g_marker_pose_wrt_arm_base.pose.position.z;
    g_quat.x() = g_marker_pose_wrt_arm_base.pose.orientation.x;
    g_quat.y() = g_marker_pose_wrt_arm_base.pose.orientation.y;
    g_quat.z() = g_marker_pose_wrt_arm_base.pose.orientation.z;
    g_quat.w() = g_marker_pose_wrt_arm_base.pose.orientation.w;   
    g_R = g_quat.matrix();      


}

bool appendPoseService(cwru_srv::path_service_messageRequest& request, cwru_srv::path_service_messageResponse& response) {
    geometry_msgs::PoseStamped pose;
    //double x, y, phi;
    geometry_msgs::Quaternion quaternion;
    ROS_INFO("service append-Path callback activated");
    /* Path message:
     * #An array of poses that represents a Path for a robot to follow
        Header header
        geometry_msgs/PoseStamped[] poses
     */
    g_marker_pose_in.header = request.path.poses[0].header;
    g_marker_pose_in.pose=request.path.poses[0].pose;
    g_marker_pose_in.header.stamp=ros::Time::now();
    g_tfListener->transformPose("link1", g_marker_pose_in, g_marker_pose_wrt_arm_base);
    ROS_INFO("Transformation Went Through...");

    g_p[0] = g_marker_pose_wrt_arm_base.pose.position.x;
    g_p[1] = g_marker_pose_wrt_arm_base.pose.position.y;
    g_p[2] = g_marker_pose_wrt_arm_base.pose.position.z;
    g_quat.x() = g_marker_pose_wrt_arm_base.pose.orientation.x;
    g_quat.y() = g_marker_pose_wrt_arm_base.pose.orientation.y;
    g_quat.z() = g_marker_pose_wrt_arm_base.pose.orientation.z;
    g_quat.w() = g_marker_pose_wrt_arm_base.pose.orientation.w;   
    g_R = g_quat.matrix();     
    


    g_marker_pose_in.header = request.path.poses[0].header;
    g_marker_pose_in.pose=request.path.poses[0].pose;
    g_marker_pose_in.header.stamp=ros::Time::now();
    g_marker_pose_in.pose.position.z = g_marker_pose_in.pose.position.z + 0.2; //Here we added a transition trajectory 20 cm above the can

    g_tfListener->transformPose("link1", g_marker_pose_in, g_marker_pose_wrt_arm_base);
    ROS_INFO("Transformation Went Through...");

    g_p_above[0] = g_marker_pose_wrt_arm_base.pose.position.x;
    g_p_above[1] = g_marker_pose_wrt_arm_base.pose.position.y;
    g_p_above[2] = g_marker_pose_wrt_arm_base.pose.position.z;
    g_R_above = g_R;
    response.resp = true; // boring, but valid response info
    return true;
}

void jointStateCB(
const sensor_msgs::JointStatePtr &js_msg) {
    
    for (int i=0;i<6;i++) {
        g_q_state[i] = js_msg->position[i];
    }
    //cout<<"g_q_state: "<<g_q_state.transpose()<<endl;
    
}

bool triggerService(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response)
{
    ROS_INFO("service callback activated");
    response.resp = true; // boring, but valid response info
    // grab the most recent IM data and repackage it as an Affine3 matrix to set a target hand pose;
    g_A_flange_desired.translation() = g_p;
    g_A_flange_desired.linear() = g_R;
    
    g_A_flange_desired_a.translation() = g_p_above;
    g_A_flange_desired_a.linear() = g_R_above;
    
    cout<<"g_p: "<<g_p.transpose()<<endl;
    cout<<"R: "<<endl;
    cout<<g_R<<endl;
    g_trigger=true; //inform "main" that we have a new goal!
    return true;
}

//command robot to move to "qvec" using a trajectory message, sent via ROS-I
void stuff_trajectory( Vectorq6x1 qvec, trajectory_msgs::JointTrajectory &new_trajectory) {
    
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    trajectory_msgs::JointTrajectoryPoint trajectory_point2; 
     
    
    new_trajectory.points.clear();
    new_trajectory.joint_names.push_back("joint_1");
    new_trajectory.joint_names.push_back("joint_2");
    new_trajectory.joint_names.push_back("joint_3");
    new_trajectory.joint_names.push_back("joint_4");
    new_trajectory.joint_names.push_back("joint_5");
    new_trajectory.joint_names.push_back("joint_6");   

    new_trajectory.header.stamp = ros::Time::now();  
    
    trajectory_point1.positions.clear();    
    trajectory_point2.positions.clear(); 
    //fill in the points of the trajectory: initially, all home angles
    for (int ijnt=0;ijnt<6;ijnt++) {
        trajectory_point1.positions.push_back(g_q_state[ijnt]); // stuff in position commands for 6 joints
        //should also fill in trajectory_point.time_from_start
        trajectory_point2.positions.push_back(0.0); // stuff in position commands for 6 joints        
    }
    trajectory_point1.time_from_start =    ros::Duration(0);  
    trajectory_point2.time_from_start =    ros::Duration(2.0);      

    // start from home pose... really, should should start from current pose!
    new_trajectory.points.push_back(trajectory_point1); // add this single trajectory point to the trajectory vector   
    new_trajectory.points.push_back(trajectory_point2); // quick hack--return to home pose
    
    // fill in the target pose: really should fill in a sequence of poses leading to this goal
   trajectory_point2.time_from_start =    ros::Duration(4.0);  
    for (int ijnt=0;ijnt<6;ijnt++) {
            trajectory_point2.positions[ijnt] = qvec[ijnt];
    }  

    new_trajectory.points.push_back(trajectory_point2); // append this point to trajectory
}


//command robot to move to "qvec" using a trajectory message, sent via ROS-I
void stuff_trajectory_epsilon( Vectorq6x1 qvec, trajectory_msgs::JointTrajectory &new_trajectory, 
        int nsolns, Vectorq6x1 qvec_above) {
    
    //trajectory_msgs::JointTrajectoryPoint trajectory_point_h;
    trajectory_msgs::JointTrajectoryPoint trajectory_point_g; //goal
    trajectory_msgs::JointTrajectoryPoint trajectory_point_t; //transition
    trajectory_msgs::JointTrajectoryPoint trajectory_point_c; //current
    trajectory_msgs::JointTrajectoryPoint trajectory_point_a; //above can
    //Vectorq6x1 qvec; 
    //Vectorq6x1 home_state;
    
    new_trajectory.points.clear();
    new_trajectory.joint_names.clear();
    new_trajectory.joint_names.push_back("joint_1");
    new_trajectory.joint_names.push_back("joint_2");
    new_trajectory.joint_names.push_back("joint_3");
    new_trajectory.joint_names.push_back("joint_4");
    new_trajectory.joint_names.push_back("joint_5");
    new_trajectory.joint_names.push_back("joint_6");   

    new_trajectory.header.stamp = ros::Time::now(); 
    
    
    trajectory_point_c.positions.clear();    
    trajectory_point_t.positions.clear(); 
    trajectory_point_g.positions.clear();
    trajectory_point_a.positions.clear();
    //fill in the points of the trajectory: initially, all home angles
    for (int ijnt=0;ijnt<6;ijnt++) {
        trajectory_point_c.positions.push_back(g_q_state[ijnt]); // stuff in position commands for 6 joints
        //should also fill in trajectory_point.time_from_start
        trajectory_point_t.positions.push_back(trans_state[ijnt]); // stuff in position commands for 6 joints   
        //trajectory_point2.positions[ijnt] = qvec[ijnt]; //put in final position command
	trajectory_point_g.positions.push_back(qvec[ijnt]);
        trajectory_point_a.positions.push_back(qvec_above[ijnt]);
    }
    trajectory_point_c.time_from_start =    ros::Duration(0);  
    trajectory_point_t.time_from_start =    ros::Duration(2.0);      
    trajectory_point_a.time_from_start =    ros::Duration(8.0);

    // start from home pose... really, should should start from current pose!
    new_trajectory.points.push_back(trajectory_point_c); // add this single trajectory point to the trajectory vector   
    new_trajectory.points.push_back(trajectory_point_t); // quick hack--return to home pose
    new_trajectory.points.push_back(trajectory_point_a); //The 
    // fill in the target pose: really should fill in a sequence of poses leading to this goal
    trajectory_point_g.time_from_start =    ros::Duration(18.0); 
    //trajectory_point_t.time_from_start =    ros::Duration(30.0); 
    new_trajectory.points.push_back(trajectory_point_g); // append this point to trajectory
    //new_trajectory.points.push_back(trajectory_point_t);

    
}

void stuff_trajectory_epsilon_back(trajectory_msgs::JointTrajectory &new_trajectory, int nsolns) {

    trajectory_msgs::JointTrajectoryPoint trajectory_point_t; //transition
    trajectory_msgs::JointTrajectoryPoint trajectory_point_c; //current

    new_trajectory.points.clear();
    new_trajectory.joint_names.clear();
    new_trajectory.joint_names.push_back("joint_1");
    new_trajectory.joint_names.push_back("joint_2");
    new_trajectory.joint_names.push_back("joint_3");
    new_trajectory.joint_names.push_back("joint_4");
    new_trajectory.joint_names.push_back("joint_5");
    new_trajectory.joint_names.push_back("joint_6");   

    new_trajectory.header.stamp = ros::Time::now(); 
    
    
    trajectory_point_c.positions.clear();    
    trajectory_point_t.positions.clear(); 

    //fill in the points of the trajectory: initially, all home angles
    for (int ijnt=0;ijnt<6;ijnt++) {
        trajectory_point_c.positions.push_back(g_q_state[ijnt]); // stuff in position commands for 6 joints
        //should also fill in trajectory_point.time_from_start
        trajectory_point_t.positions.push_back(trans_state[ijnt]); // stuff in position commands for 6 joints   
    }
    trajectory_point_c.time_from_start =    ros::Duration(0);  
    trajectory_point_t.time_from_start =    ros::Duration(2.0);      

    // start from home pose... really, should should start from current pose!
    new_trajectory.points.push_back(trajectory_point_c); // add this single trajectory point to the trajectory vector   
    new_trajectory.points.push_back(trajectory_point_t); // quick hack--return to home pose  
}

int checkSolnsToAbove(std::vector<Vectorq6x1> all_qvec, int nsolns) {
    double all_diff[nsolns];
    for (int i = 0; i < nsolns; i++) { 
       double diff = 0;
       Vectorq6x1 qvec = all_qvec[i];
       for(int j = 0; j < 6; j++) {
           diff = diff + trans_state[j] - qvec[j];
       }
       all_diff[i] = diff;
       ROS_WARN("all_diff[%d] = %f", i, all_diff[i]);
    }
    double temp_diff = fabs(all_diff[0]);
    int temp_i = 0;
    for (int i = 1; i < nsolns; i++) {
        if (fabs(all_diff[i]) < temp_diff) {
            temp_i = i;
            temp_diff = fabs(all_diff[i]);
        }
    }
    return temp_i;
}

int checkSolnsToGoal(std::vector<Vectorq6x1> a_qvec, std::vector<Vectorq6x1> g_qvec , int g_nsolns, int a_chosen ) {
    double all_diff[g_nsolns];
    Vectorq6x1 a_qvec1 = a_qvec[a_chosen];
    for (int i = 0; i < g_nsolns; i++) { 
       double diff = 0;
       //Vectorq6x1 qvec1a = a_qvec[i];
       Vectorq6x1 qvec1g = g_qvec[i];
       for(int j = 0; j < 6; j++) {
           diff = diff + a_qvec1[j] - qvec1g[j];
       }
       all_diff[i] = diff;
       ROS_WARN("all_diff[%d] = %f", i, all_diff[i]);
    }
    double temp_diff = fabs(all_diff[0]);
    int temp_i = 0;
    for (int i = 1; i < g_nsolns; i++) {
        if (fabs(all_diff[i]) < temp_diff) {
            temp_i = i;
            temp_diff = fabs(all_diff[i]);
        }
    }
    return temp_i;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_marker_listener"); // this will be the node name;
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);  
    ROS_INFO("setting up subscribers ");
    ros::Subscriber sub_js = nh.subscribe("/joint_states",1,jointStateCB);
    ros::Subscriber sub_im = nh.subscribe("example_marker/feedback", 1, markerListenerCB);
    ros::ServiceServer service = nh.advertiseService("move_trigger", triggerService);   
    
    ros::ServiceServer append_path_ = nh.advertiseService("appendPoseService", appendPoseService);
    
    Eigen::Vector3d p;
    Eigen::Vector3d n_des,t_des,b_des;
    std::vector<Vectorq6x1> q6dof_solns, q6dof_solns_a;
    Vectorq6x1 qvec, qvec_above;
    ros::Rate sleep_timer(10.0); //10Hz update rate    
    Irb120_fwd_solver irb120_fwd_solver; //instantiate forward and IK solvers
    Irb120_IK_solver ik_solver, ik_solver_a;
    Eigen::Vector3d n_urdf_wrt_DH,t_urdf_wrt_DH,b_urdf_wrt_DH;
    bool begin = true;
    //Home For Abby Sim    
    //home_state[0] = 0.0245981;
    //home_state[1] = -1.43767;
    //home_state[2] = 0.0514272;
    //home_state[3] = -0.13549;
    //home_state[4] = -0.363732;
    //home_state[5] = -0.0716817;
	

// Left arm home position: [-1.5400700569152832, -1.8655591011047363, -0.4005431830883026, -0.04174210503697395, -0.8261124491691589, 1.5975639820098877]
    home_state[0] = -1.5400700569152832;
    home_state[1] = -1.8655591011047363;
    home_state[2] = -0.4005431830883026;
    home_state[3] = -0.04174210503697395;
    home_state[4] = -0.8261124491691589;
    home_state[5] = 1.5975639820098877;
    
    //Transition Pose for Physical Abby, the actual robot...
    trans_state[0] = 5.248869570095849e-07;
    trans_state[1] = -0.8018337488174438;
    trans_state[2] = -1.08710777759552;
    trans_state[3] = -5.774702458438696e-07;
    trans_state[4] = 0.31814315915107727;
    trans_state[5] = -1.5014468601748376e-07;
    // in home pose, R_urdf = I
    //DH-defined tool-flange axes point as:
    // z = 1,0,0
    // x = 0,0,-1
    // y = 0,1,0
    // but URDF frame is R = I
    // so, x_urdf_wrt_DH = z_DH = [0;0;1]
    // y_urdf_wrt_DH = y_DH = [0;1;0]
    // z_urdf_wrt_DH = -x_DH = [-1; 0; 0]
    // so, express R_urdf_wrt_DH as:
    n_urdf_wrt_DH <<0,0,1;
    t_urdf_wrt_DH <<0,1,0;
    b_urdf_wrt_DH <<-1,0,0;
    Eigen::Matrix3d R_urdf_wrt_DH;
    R_urdf_wrt_DH.col(0) = n_urdf_wrt_DH;
    R_urdf_wrt_DH.col(1) = t_urdf_wrt_DH;
    R_urdf_wrt_DH.col(2) = b_urdf_wrt_DH;    

    trajectory_msgs::JointTrajectory new_trajectory, back_trajectory; // an empty trajectory

    
    //qvec<<0,0,0,0,0,0;
    Eigen::Affine3d A_flange_des_DH, A_flange_des_DH_a;
    
    //   A_fwd_DH = irb120_fwd_solver.fwd_kin_solve(qvec); //fwd_kin_solve

    //std::cout << "A rot: " << std::endl;
    //std::cout << A_fwd_DH.linear() << std::endl;
    //std::cout << "A origin: " << A_fwd_DH.translation().transpose() << std::endl;   
    
    
    g_tfListener = new tf::TransformListener;  //create a transform listener
    
    // wait to start receiving valid tf transforms between map and odom:
    bool tferr=true;
    ROS_INFO("waiting for tf between base_link and link1 of arm...");
    while (tferr) {
        tferr=false;
        try {
            //try to lookup transform from target frame "odom" to source frame "map"
            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
                g_tfListener->lookupTransform("base_link", "link1", ros::Time(0), g_armlink1_wrt_baseLink);
            } catch(tf::TransformException &exception) {
                ROS_ERROR("%s", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();                
            }   
    }
    ROS_INFO("tf is good");
    // from now on, tfListener will keep track of transforms        
  
    
    int nsolns;
    
    while(ros::ok()) {
            ros::spinOnce();
            if (g_trigger) {
                // ooh!  excitement time!  got a new tool pose goal!
                g_trigger=false; // reset the trigger
                //is this point reachable?
                A_flange_des_DH = g_A_flange_desired;
                A_flange_des_DH.linear() = g_A_flange_desired.linear()*R_urdf_wrt_DH.transpose();
                
                A_flange_des_DH_a = g_A_flange_desired_a;
                A_flange_des_DH_a.linear() = g_A_flange_desired_a.linear()*R_urdf_wrt_DH.transpose();
                
                cout<<"R des DH: "<<endl;
                cout<<A_flange_des_DH.linear()<<endl;
		int nsolns_a = ik_solver_a.ik_solve(A_flange_des_DH_a);
		ROS_INFO("there are %d solutions above",nsolns_a);
                nsolns = ik_solver.ik_solve(A_flange_des_DH);
                ROS_INFO("there are %d solutions",nsolns);

                if (nsolns>0) {      
                    ik_solver.get_solns(q6dof_solns);
                    ik_solver_a.get_solns(q6dof_solns_a);
                    //qvec = checkSolns(q6dof_solns, nsolns);
                    int soln_a = checkSolnsToAbove(q6dof_solns_a, nsolns_a);
                    
		    int soln_g = checkSolnsToGoal(q6dof_solns_a, q6dof_solns , nsolns, soln_a);
                    ROS_WARN("Chosen Solution = %d and %d", soln_g, soln_a);
                    //ROS_WARN("qvec[%d] = %f", ijnt, qvec[ijnt]);
		    qvec_above = q6dof_solns_a[soln_a];
                    qvec = q6dof_solns[soln_a]; // choose the best solution

		ROS_WARN("Done For Here");
                    // stuff_trajectory(qvec,new_trajectory); 
                    stuff_trajectory_epsilon(qvec, new_trajectory, nsolns, qvec_above);
                    
                    pub.publish(new_trajectory);
                    ros::Duration(80.0).sleep();
		    ros::spinOnce();
		    stuff_trajectory_epsilon_back(back_trajectory, nsolns);
                    pub.publish(back_trajectory);
                }
            }
            if (begin) {
                begin=false;
                trajectory_msgs::JointTrajectoryPoint trajectory_point_h;
                trajectory_msgs::JointTrajectoryPoint trajectory_point_c;
                ROS_WARN("First Time...");
                new_trajectory.points.clear();
                new_trajectory.joint_names.clear();
                new_trajectory.joint_names.push_back("joint_1");
                new_trajectory.joint_names.push_back("joint_2");
                new_trajectory.joint_names.push_back("joint_3");
                new_trajectory.joint_names.push_back("joint_4");
                new_trajectory.joint_names.push_back("joint_5");
                new_trajectory.joint_names.push_back("joint_6");   
                new_trajectory.header.stamp = ros::Time::now();
                
                trajectory_point_c.positions.clear();
                trajectory_point_h.positions.clear();
                for (int ijnt=0;ijnt<6;ijnt++) {
                    trajectory_point_c.positions.push_back(g_q_state[ijnt]); // stuff in position commands for 6 joints
                    //should also fill in trajectory_point.time_from_start
                    trajectory_point_h.positions.push_back(home_state[ijnt]); // stuff in position commands for 6 joints   
                    //trajectory_point2.positions[ijnt] = qvec[ijnt]; //put in final position command
                }
                trajectory_point_c.time_from_start =    ros::Duration(0);  
                trajectory_point_h.time_from_start =    ros::Duration(2.0);      

                // start from home pose... really, should should start from current pose!
                new_trajectory.points.push_back(trajectory_point_c); // add this single trajectory point to the trajectory vector   
                new_trajectory.points.push_back(trajectory_point_h); // quick hack--return to home pose
                pub.publish(new_trajectory);
            }
            sleep_timer.sleep();    
            
    }
    
    return 0;
}


