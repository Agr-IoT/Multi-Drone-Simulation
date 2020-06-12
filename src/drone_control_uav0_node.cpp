#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <sstream>
#include <string>

mavros_msgs::State current_state;
nav_msgs::Odometry current_pos;
bool reached = false;

void state_callback(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}

void position_callback(const nav_msgs::Odometry::ConstPtr& msg){
  current_pos = *msg;
}


void sensorCallback(const std_msgs::String::ConstPtr& msg)
{
    float curr_x_pose = current_pos.pose.pose.position.x;
    float curr_y_pose = current_pos.pose.pose.position.y;
    if(curr_y_pose< -0.8 && curr_y_pose > -1.2 &&
      curr_x_pose  < 5.2 && curr_x_pose > 4.8){
      ROS_INFO("I got data : [%s]", msg->data.c_str());
    }
    if(curr_y_pose< -2.7 && curr_y_pose > -3.3 &&
      curr_x_pose  < 9.2 && curr_x_pose > 8.8){
      ROS_INFO("I got data : [%s]", msg->data.c_str());
    }

    if(curr_y_pose< -5.8 && curr_y_pose > -6.2 &&
      curr_x_pose  < 12.2 && curr_x_pose > 10.8){
      ROS_INFO("I got data : [%s]", msg->data.c_str());
    }

    if(curr_y_pose< -7.8 && curr_y_pose > -8.2 &&
      curr_x_pose  < 3.2 && curr_x_pose > 2.8){
      ROS_INFO("I got data : [%s]", msg->data.c_str());
    }


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_node_uav0");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    ("/uav0/mavros/state", 10, state_callback);
  ros::Subscriber position_sub = nh.subscribe<nav_msgs::Odometry>
      ("/uav0/mavros/global_position/local", 10, position_callback);
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
  for(int i = 100; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;


  geometry_msgs::PoseStamped sensor1Position;
  sensor1Position.pose.position.x = 5;
  sensor1Position.pose.position.y = -1;
  sensor1Position.pose.position.z = 2;

  geometry_msgs::PoseStamped sensor2Position;
  sensor2Position.pose.position.x = 9;
  sensor2Position.pose.position.y = -3;
  sensor2Position.pose.position.z = 2;

  geometry_msgs::PoseStamped sensor3Position;
  sensor3Position.pose.position.x = 12;
  sensor3Position.pose.position.y = -6;
  sensor3Position.pose.position.z = 3;

  geometry_msgs::PoseStamped sensor4Position;
  sensor4Position.pose.position.x = 3;
  sensor4Position.pose.position.y = -8;
  sensor4Position.pose.position.z = 2;

  geometry_msgs::PoseStamped sensor5Position;
  sensor5Position.pose.position.x = 7;
  sensor5Position.pose.position.y = -8;
  sensor5Position.pose.position.z = 3;

  geometry_msgs::PoseStamped sensor6Position;
  sensor6Position.pose.position.x = 3;
  sensor6Position.pose.position.y = -7;
  sensor6Position.pose.position.z = 2;

  geometry_msgs::PoseStamped sensor7Position;
  sensor7Position.pose.position.x = 1;
  sensor7Position.pose.position.y = -7;
  sensor7Position.pose.position.z = 1;

  geometry_msgs::PoseStamped sensor8Position;
  sensor8Position.pose.position.x = 1;
  sensor8Position.pose.position.y = -3;
  sensor8Position.pose.position.z = 2;

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

        for(int i = 100; ros::ok() && i > 0; --i){
          local_pos_pub.publish(pose);
          ros::spinOnce();
          rate.sleep();
        }
      }
      else if(countPosition == 1){
        ros::Subscriber sub = nh.subscribe("sensor1_data", 500, sensorCallback);
        reached = true;
        std::cout << "enter into if counter " + std::to_string(countPosition) << '\n';
        //std::cout << "enter into if counter 1" + std::to_string(current_pos.pose.pose.position.y) << '\n';

        for(int i = 100; ros::ok() && i > 0; --i){
          local_pos_pub.publish(sensor1Position);
          ros::spinOnce();
          rate.sleep();
        }

        std::cout << "exit from if counter " + std::to_string(countPosition) << '\n';
        //std::cout << "enter into if counter 1" + std::to_string(current_pos.pose.pose.position.y) << '\n';
      }

      else if(countPosition == 2){
        ros::Subscriber sub = nh.subscribe("sensor2_data", 500, sensorCallback);
        std::cout << "enter into if counter " + std::to_string(countPosition) << '\n';
        for(int i = 100; ros::ok() && i > 0; --i){
          local_pos_pub.publish(sensor2Position);
          ros::spinOnce();
          rate.sleep();
        }
        std::cout << "exit from if counter " + std::to_string(countPosition) << '\n';
      }

      else if(countPosition == 3){
        ros::Subscriber sub = nh.subscribe("sensor3_data", 500, sensorCallback);
        std::cout << "enter into if counter " + std::to_string(countPosition) << '\n';
        for(int i = 100; ros::ok() && i > 0; --i){
          local_pos_pub.publish(sensor3Position);
          ros::spinOnce();
          rate.sleep();
        }
        std::cout << "exit from if counter " + std::to_string(countPosition) << '\n';
      }
      else if(countPosition == 4){
        std::cout << "enter into if counter " + std::to_string(countPosition) << '\n';
        for(int i = 100; ros::ok() && i > 0; --i){
          local_pos_pub.publish(sensor4Position);
          ros::spinOnce();
          rate.sleep();
        }
        std::cout << "exit from if counter " + std::to_string(countPosition) << '\n';

      }
      else if(countPosition == 5){
        std::cout << "enter into if counter " + std::to_string(countPosition) << '\n';
        for(int i = 100; ros::ok() && i > 0; --i){
          local_pos_pub.publish(sensor5Position);
          ros::spinOnce();
          rate.sleep();
        }
        std::cout << "exit from if counter " + std::to_string(countPosition) << '\n';
      }
      else if(countPosition == 6){
        std::cout << "enter into if counter " + std::to_string(countPosition) << '\n';
        for(int i = 100; ros::ok() && i > 0; --i){
          local_pos_pub.publish(sensor6Position);
          ros::spinOnce();
          rate.sleep();
        }
        std::cout << "exit from if counter " + std::to_string(countPosition) << '\n';
      }
      else if(countPosition == 7){
        std::cout << "enter into if counter  " + std::to_string(countPosition) << '\n';
        for(int i = 100; ros::ok() && i > 0; --i){
          local_pos_pub.publish(sensor7Position);
          ros::spinOnce();
          rate.sleep();
        }
        std::cout << "exit from if counter " + std::to_string(countPosition) << '\n';
      }

      else if(countPosition == 8){
        std::cout << "enter into if counter " + std::to_string(countPosition) << '\n';
        for(int i = 100; ros::ok() && i > 0; --i){
          local_pos_pub.publish(sensor8Position);
          ros::spinOnce();
          rate.sleep();
        }
        std::cout << "exit from if counter " + std::to_string(countPosition) << '\n';
      }





      if(current_state.armed && ros::Time::now() - time_start1 > ros::Duration(8.0)){
        //std::cout << "position is" + std::to_string(countPosition) << '\n';
        std::cout << "current_pos y = " + std::to_string(current_pos.pose.pose.position.y) << '\n';
        countPosition += 1  ;
        reached = false;
        if(countPosition > 4){
          countPosition = 0;
          ROS_INFO("uav0/countPosition 0"  );
          time_start1 = ros::Time::now();
        }
      }

      //local_pos_pub.publish(pose);

      //local_vel_pub.publish(vel);



      ros::spinOnce();
      rate.sleep();
    }

  return 0;
}
