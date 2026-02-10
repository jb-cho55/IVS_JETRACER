#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "jetRacer_ROS_Wrap";

// For Block jetRacer_ROS_Wrap/Subscribe
SimulinkSubscriber<std_msgs::Float32, SL_Bus_jetRacer_ROS_Wrap_std_msgs_Float32> Sub_jetRacer_ROS_Wrap_4;

// For Block jetRacer_ROS_Wrap/Publish
SimulinkPublisher<std_msgs::Float32, SL_Bus_jetRacer_ROS_Wrap_std_msgs_Float32> Pub_jetRacer_ROS_Wrap_2;

// For Block jetRacer_ROS_Wrap/Publish1
SimulinkPublisher<std_msgs::Float32, SL_Bus_jetRacer_ROS_Wrap_std_msgs_Float32> Pub_jetRacer_ROS_Wrap_14;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

