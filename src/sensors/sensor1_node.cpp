#include <ros/ros.h>
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
  ros::init(argc, argv, "sensor1_node");

  // Create a ROS node handle
  ros::NodeHandle nh;

  ROS_INFO("sensor1  Initialized");

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

   ros::Rate loop_rate(1);// it's enough for our context

   /**
    * A count of how many messages we have sent. This is used to create
    * a unique string for each message.
    */
   int count = 0;
   while (ros::ok())
   {
     /**
      * This is a message object. You stuff it with data, and then publish it.
      */
     std_msgs::String msg;

     std::stringstream ss;
     ss << "sensor1 data " << count;
     msg.data = ss.str();

     //ROS_INFO("%s", msg.data.c_str());

     chatter_pub.publish(msg);

     ros::spinOnce();

     loop_rate.sleep();
     ++count;
   }

  // Don't exit the program.
  ros::spin();
}
