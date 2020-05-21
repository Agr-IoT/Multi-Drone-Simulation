#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <string>

mavros_msgs::State current_state;

void state_callback(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_node_feedback");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    ("/uav0/mavros/state", 10, state_callback);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    ("/uav0/mavros/setpoint_position/local", 10);
  ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
    ("/uav0/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    ("/uav0/mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    ("/uav0/mavros/set_mode");

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  ROS_INFO("Initializing...");
  // wait for FCU connection
  while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("Connected.");

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 2;


  geometry_msgs::Twist vel;
  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;

  //send a few setpoints before starting
  for(int i = 100;loop_time_start ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;


  geometry_msgs::PoseStamped pose1;
  pose.pose.position.x = 2;
  pose.pose.position.y = 2;
  pose.pose.position.z = 2;

  ros::Time last_request = ros::Time::now();

  ros::Time time_start = ros::Time::now();
  ros::Time time_start1 = ros::Time::now();
  bool isArnmed = false;
  int  countPosition = 0;
  while(ros::ok()){

      ros::Time loop_time_start = ros::Time::now();
      if( current_state.mode != "OFFBOARD" &&
        	(ros::Time::now() - last_request > ros::Duration(5.0))){
              if( set_mode_client.call(offb_set_mode) &&
                	  offb_set_mode.response.mode_sent){
                   	ROS_INFO("Offboard enabled");

              }
              last_request = ros::Time::now();
      } else {
              if( !current_state.armed &&
        	  (ros::Time::now() - last_request > ros::Duration(5.0))){
                	if( arming_client.call(arm_cmd) &&
                	    arm_cmd.response.success){
                	  ROS_INFO("Vehicle armed");
                    time_start1 = ros::Time::now();
                	}
          	       last_request = ros::Time::now();
              }
      }

      // Update the desired pose:
      /*pose.pose.position.x = 9*sin(2.0*M_PI*0.3*(ros::Time::now()-time_start).toSec());
      pose.pose.position.y = 9*cos(2.0*M_PI*0.3*(ros::Time::now()-time_start).toSec());

      //Update the desired velocity:
      vel.linear.x = 4.0*M_PI*0.1*cos(2.0*M_PI*0.1*(ros::Time::now()-time_start).toSec());
      vel.linear.y = -4.0*M_PI*0.1*sin(2.0*M_PI*0.1*(ros::Time::now()-time_start).toSec());*/
      if(countPosition == 0){
        local_pos_pub.publish(pose);
      }
      if(current_state.armed && ros::Time::now() - time_start1 > ros::Duration(15.0)){
        countPosition += 1  ;
        if(countPosition == 2){
          countPosition = 0;
          ROS_INFO("countPosition 0"  );
          time_start1 = ros::Time::now();
        }
      }

      while(current_state.armed &&  ros::Time::now() - time_start1 < ros::Duration(30.0) &&
            ros::Time::now() - time_start1 > ros::Duration(15.0) &&
            countPosition == 1){
        local_pos_pub.publish(pose1);

      }

      //local_vel_pub.publish(vel);



      ros::spinOnce();
      rate.sleep();
    }

  return 0;
}
