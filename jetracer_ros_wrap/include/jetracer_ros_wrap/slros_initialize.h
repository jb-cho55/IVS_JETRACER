#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"
#include "jetRacer_ROS_Wrap_types.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block jetRacer_ROS_Wrap/Subscribe
extern SimulinkSubscriber<std_msgs::Float32, SL_Bus_jetRacer_ROS_Wrap_std_msgs_Float32> Sub_jetRacer_ROS_Wrap_4;

// For Block jetRacer_ROS_Wrap/Publish
extern SimulinkPublisher<std_msgs::Float32, SL_Bus_jetRacer_ROS_Wrap_std_msgs_Float32> Pub_jetRacer_ROS_Wrap_2;

// For Block jetRacer_ROS_Wrap/Publish1
extern SimulinkPublisher<std_msgs::Float32, SL_Bus_jetRacer_ROS_Wrap_std_msgs_Float32> Pub_jetRacer_ROS_Wrap_14;

void slros_node_init(int argc, char** argv);

#endif
