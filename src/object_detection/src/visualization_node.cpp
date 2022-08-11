// ROS lib
#include <ros/ros.h>

// local lib
#include "object_detection/visualization.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualization_node");
  ros::NodeHandle nh;

  Visualize3DBox gps_node(nh);

  ros::spin();

  return 0;
}
