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
#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <sensor_msgs/JointState.h>

//callback to subscribe to marker state
Eigen::Vector3d g_p;
Vectorq6x1 g_q_state;
Vectorq6x1 home_state; //= {0.0245981, -1.43767, 0.0514272, -0.13549, -0.363732, -0.0716817};

double g_x,g_y,g_z;
//geometry_msgs::Quaternion g_quat; // global var for quaternion
Eigen::Quaterniond g_quat;
Eigen::Matrix3d g_R;
Eigen::Affine3d g_A_flange_desired;
bool g_trigger=false;
using namespace std;

void markerListenerCB(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    //copy to global vars:
    g_p[0] = feedback->pose.position.x;
    g_p[1] = feedback->pose.position.y;
    g_p[2] = feedback->pose.position.z;
    g_quat.x() = feedback->pose.orientation.x;
    g_quat.y() = feedback->pose.orientation.y;
    g_quat.z() = feedback->pose.orientation.z;
    g_quat.w() = feedback->pose.orientation.w;   
    g_R = g_quat.matrix();
}

void jointStateCB(
const sensor_msgs::JointStatePtr &js_msg) {
    
    for (int i=0;i<6;i++) {
        g_q_state[i] = js_msg->position[i];
    }
    cout<<"g_q_state: "<<g_q_state.transpose()<<endl;
    
}

bool triggerService(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response)
{
    ROS_INFO("service callback activated");
    response.resp = true; // boring, but valid response info
    // grab the most recent IM data and repackage it as an Affine3 matrix to set a target hand pose;
    g_A_flange_desired.translation() = g_p;
    g_A_flange_desired.linear() = g_R;
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
        //trajectory_point2.positions[ijnt] = qvec[ijnt]; //put in final position command
    }
    trajectory_point1.time_from_start =    ros::Duration(0);  
    trajectory_point2.time_from_start =    ros::Duration(2.0);      

    // start from home pose... really, should should start from current pose!
    new_trajectory.points.push_back(trajectory_point1); // add this single trajectory point to the trajectory vector   
    //new_trajectory.points.push_back(trajectory_point2); // quick hack--return to home pose
    
    // fill in the target pose: really should fill in a sequence of poses leading to this goal
   trajectory_point2.time_from_start =    ros::Duration(4.0);  
    for (int ijnt=0;ijnt<6;ijnt++) {
            trajectory_point2.positions[ijnt] = qvec[ijnt];
    }  

    new_trajectory.points.push_back(trajectory_point2); // append this point to trajectory
}

//command robot to move to "qvec" using a trajectory message, sent via ROS-I
void stuff_trajectory_epsilon( std::vector<Vectorq6x1> all_qvec, trajectory_msgs::JointTrajectory &new_trajectory, int nsolns) {
    
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    trajectory_msgs::JointTrajectoryPoint trajectory_point2; 
    Vectorq6x1 qvec; 
    //Vectorq6x1 home_state;
    
    new_trajectory.points.clear();
    new_trajectory.joint_names.push_back("joint_1");
    new_trajectory.joint_names.push_back("joint_2");
    new_trajectory.joint_names.push_back("joint_3");
    new_trajectory.joint_names.push_back("joint_4");
    new_trajectory.joint_names.push_back("joint_5");
    new_trajectory.joint_names.push_back("joint_6");   

    new_trajectory.header.stamp = ros::Time::now();  
  
    // home state for ABBY's arm extended left  
   //    home_state[0] = -1.56675;
// //    home_state[1] = -1.40744;
// //    home_state[2] = -1.54781;
// //    home_state[3] = 0.0153577;
// //    home_state[4] = -0.206403;
// //    home_state[5] = -1.59364;  
    
// //    home_state[0] = 0.0245981;
// //    home_state[1] = -1.43767;
// //    home_state[2] = 0.0514272;
// //    home_state[3] = -0.13549;
// //    home_state[4] = -0.363732;
// //    home_state[5] = -0.0716817;
    
    //home state for ABBY in gazebo
    //home_state[0] = -.226703;
    //home_state[1] = 0.45006;
    //home_state[2] = -1.62978;
    //home_state[3] = 0.259641;
    //home_state[4] = 7.44516;
    //home_state[5] = 0.819829;
    //Home position for ABBY (not gazebo)
    // 0.0245981   -1.43767  0.0514272   -0.13549  -0.363732 -0.0716817
    
    
    trajectory_point1.positions.clear();    
    trajectory_point2.positions.clear(); 
    //fill in the points of the trajectory: initially, all home angles
    for (int ijnt=0;ijnt<6;ijnt++) {
        trajectory_point1.positions.push_back(g_q_state[ijnt]); // stuff in position commands for 6 joints
        //should also fill in trajectory_point.time_from_start
        trajectory_point2.positions.push_back(home_state[ijnt]); // stuff in position commands for 6 joints   
        //trajectory_point2.positions[ijnt] = qvec[ijnt]; //put in final position command
    }
    trajectory_point1.time_from_start =    ros::Duration(0);  
    trajectory_point2.time_from_start =    ros::Duration(2.0);      

    // start from home pose... really, should should start from current pose!
    new_trajectory.points.push_back(trajectory_point1); // add this single trajectory point to the trajectory vector   
    new_trajectory.points.push_back(trajectory_point2); // quick hack--return to home pose
    
    // fill in the target pose: really should fill in a sequence of poses leading to this goal
   trajectory_point2.time_from_start =    ros::Duration(4.0); 
   for (int i = 0; i < nsolns; i++) {
       qvec = all_qvec[i];
       ROS_WARN("nsoln = %d", i);
         for (int ijnt=0;ijnt<6;ijnt++) {
                trajectory_point2.positions[ijnt] = qvec[ijnt];
                ROS_WARN("qvec[%d] = %f", ijnt, qvec[ijnt]);
         } 
   }

    new_trajectory.points.push_back(trajectory_point2); // append this point to trajectory
}



int checkSolns(std::vector<Vectorq6x1> all_qvec, int nsolns) {
    double all_diff[nsolns];
    for (int i = 0; i < nsolns; i++) { 
       double diff = 0;
       Vectorq6x1 qvec = all_qvec[i];
       for(int j = 0; j < 6; j++) {
           diff = diff + home_state[j] - qvec[j];
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



int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_marker_listener"); // this will be the node name;
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);  
    ROS_INFO("setting up subscribers ");
    ros::Subscriber sub_js = nh.subscribe("/joint_states",1,jointStateCB);
    ros::Subscriber sub_im = nh.subscribe("example_marker/feedback", 1, markerListenerCB);
    ros::ServiceServer service = nh.advertiseService("move_trigger", triggerService);   
    
    Eigen::Vector3d p;
    Eigen::Vector3d n_des,t_des,b_des;
    std::vector<Vectorq6x1> q6dof_solns;
    Vectorq6x1 qvec;
    ros::Rate sleep_timer(10.0); //10Hz update rate    
    Irb120_fwd_solver irb120_fwd_solver; //instantiate forward and IK solvers
    Irb120_IK_solver ik_solver;
    Eigen::Vector3d n_urdf_wrt_DH,t_urdf_wrt_DH,b_urdf_wrt_DH;
    
    home_state[0] = 0.0245981;
    home_state[1] = -1.43767;
    home_state[2] = 0.0514272;
    home_state[3] = -0.13549;
    home_state[4] = -0.363732;
    home_state[5] = -0.0716817;
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

    trajectory_msgs::JointTrajectory new_trajectory; // an empty trajectory

    
    //qvec<<0,0,0,0,0,0;
    Eigen::Affine3d A_flange_des_DH;
    
    //   A_fwd_DH = irb120_fwd_solver.fwd_kin_solve(qvec); //fwd_kin_solve

    //std::cout << "A rot: " << std::endl;
    //std::cout << A_fwd_DH.linear() << std::endl;
    //std::cout << "A origin: " << A_fwd_DH.translation().transpose() << std::endl;   
  
    
    int nsolns;
    
    while(ros::ok()) {
            ros::spinOnce();
            if (g_trigger) {
                // ooh!  excitement time!  got a new tool pose goal!
                g_trigger=false; // reset the trigger
                //is this point reachable?
                A_flange_des_DH = g_A_flange_desired;
                A_flange_des_DH.linear() = g_A_flange_desired.linear()*R_urdf_wrt_DH.transpose();
                cout<<"R des DH: "<<endl;
                cout<<A_flange_des_DH.linear()<<endl;
                nsolns = ik_solver.ik_solve(A_flange_des_DH);
                ROS_INFO("there are %d solutions",nsolns);

                if (nsolns>0) {      
                    ik_solver.get_solns(q6dof_solns);  
                    //qvec = checkSolns(q6dof_solns, nsolns);
                    int soln = checkSolns(q6dof_solns, nsolns);
                    ROS_WARN("Chosen Solution = %d", soln);
                    //ROS_WARN("qvec[%d] = %f", ijnt, qvec[ijnt]);
                    qvec = q6dof_solns[soln]; // arbitrarily choose first soln                    
                    // stuff_trajectory(qvec,new_trajectory); 
                    stuff_trajectory_epsilon(q6dof_solns, new_trajectory, nsolns);
                        pub.publish(new_trajectory);
                }
            }
            sleep_timer.sleep();    
            
    }
    
    return 0;
}


