//example ROS service:
#include <ros/ros.h>
// put this here to to test that cwru_srv messages get made... 

#include <cwru_srv/simple_bool_service_message.h>
#include <cwru_srv/simple_int_service_message.h>
#include <cwru_srv/simple_float_service_message.h>
#include <cwru_srv/IM_node_service_message.h>
#include <cwru_srv/arm_nav_service_message.h>

#include <iostream>
#include <string>
using namespace std;

bool callback(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response)
{
    ROS_INFO("callback activated");

    
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_ros_service");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("trivial_ros_service", callback);
  ROS_INFO("starting trivial ros service loop");
  ros::spin();

  return 0;
}
