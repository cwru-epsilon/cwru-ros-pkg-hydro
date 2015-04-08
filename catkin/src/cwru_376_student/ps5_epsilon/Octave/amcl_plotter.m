%AMCL
%header.seq             
%header.stamp.secs      
%header.stamp.nsecs     
%header.frame_id        
%pose.pose.position.x   
%pose.pose.position.y   
%pose.pose.position.z   
%pose.pose.orientation.x
%pose.pose.orientation.y
%pose.pose.orientation.z
%pose.pose.orientation.w
%pose.covariance.0      
%pose.covariance.1      
%pose.covariance.2      
%pose.covariance.3      
%pose.covariance.4      
%pose.covariance.5      
%pose.covariance.6      
%pose.covariance.7      
%pose.covariance.8      
%pose.covariance.9      
%pose.covariance.10     
%pose.covariance.11     
%pose.covariance.12     
%pose.covariance.13     
%pose.covariance.14     
%pose.covariance.15     
%pose.covariance.16     
%pose.covariance.17     
%pose.covariance.18     
%pose.covariance.19     
%pose.covariance.20     
%pose.covariance.21     
%pose.covariance.22     
%pose.covariance.23     
%pose.covariance.24     
%pose.covariance.25     
%pose.covariance.26     
%pose.covariance.27     
%pose.covariance.28     
%pose.covariance.29     
%pose.covariance.30     
%pose.covariance.31     
%pose.covariance.32     
%pose.covariance.33     
%pose.covariance.34     
%pose.covariance.35 

%cmd_vel_stamped
%header.seq        
%header.stamp.secs 
%header.stamp.nsecs
%header.frame_id   
%twist.linear.x    
%twist.linear.y    
%twist.linear.z    
%twist.angular.x   
%twist.angular.y   
%twist.angular.z 

%This Octave/matlab program was made to interpret amcl data from a converted bag file

clear all
load epsilonps5_amcl_pose.mat

%assemble seconds and nanoseconds to get time stamps
%subtract off starting time, so starts from t=0
%note: if load additional files, will need to use a single reference time stamp to allign
%all data to a common start time
amcl_secs=data(:,2);
amcl_nsecs=data(:,3);
amcl_time= amcl_secs+amcl_nsecs*0.000000001;
amcl_start_time = amcl_time(1)
amcl_time = amcl_time-amcl_start_time;

amcl_x = data(:,5);
amcl_y = data(:,6);
amcl_qz = data(:,10);
amcl_qw = data(:,11);
amcl_heading = 2.0*atan2(amcl_qz,amcl_qw);%cheap conversion from quaternion to heading for planar motion

%load another topic file:
load epsilonps5_cmd_vel_stamped.mat
cmd_vel_secs=data(:,2);
cmd_vel_nsecs=data(:,3);
cmd_vel_time= cmd_vel_secs+cmd_vel_nsecs*0.000000001;
cmd_vel_time = cmd_vel_time-amcl_start_time; %offset relative to amcl time, so time is aligned

cmd_vel = data(:,5);
cmd_omega = data(:,10);

figure(1)
plot(amcl_time,amcl_x,'r',amcl_time,amcl_y,'b',amcl_time,amcl_heading,'g')
xlabel('time (sec)')
ylabel('m, rad')
title('x (r), y (b) and heading (g) vs time from amcl')
grid on

figure(2)
clf
%plot(amcl_time,cmd_vel,'b',amcl_time,cmd_omega,'r')
%hold on
plot(cmd_vel_time,cmd_vel,'g',cmd_vel_time,cmd_omega,'m')
xlabel('time (sec)')
ylabel('m/sec and rad/sec')
title('speed (b) and spin (r) vs time')
grid on

figure(3)
plot(amcl_x,amcl_y)
xlabel('x (m)')
ylabel('y (m)')
title('path: x vs y')
grid

