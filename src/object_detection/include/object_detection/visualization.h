#pragma once

// ROS header
#include <ros/ros.h>
#include <message_filters/subscriber.h>

//Local header
#include "object_msgs/ObjectInfoList.h"


class Visualize3DBox
{
  public:
    Visualize3DBox(ros::NodeHandle& node);
    ~Visualize3DBox() = default;

  private:
    ros::Subscriber box_msgs_sub_;
    ros::Publisher marker_pub_;
    void box_msg_callback(const object_msgs::ObjectInfoList::ConstPtr& box_msgs);
    void generate_marker(const std::vector<object_msgs::ObjectInfo>& boxs);
};
