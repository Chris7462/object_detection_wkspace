//C++ header
#include <vector>

//ROS header
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//Local header
#include "object_detection/visualization.h"

Visualize3DBox::Visualize3DBox(ros::NodeHandle& node)
{
    box_msgs_sub_ = node.subscribe("box_msgs", 10, &Visualize3DBox::box_msg_callback, this);
    marker_pub_ = node.advertise<visualization_msgs::MarkerArray>("markers_drawing", 30, true);
}

void Visualize3DBox::box_msg_callback(const object_msgs::ObjectInfoList::ConstPtr& box_msgs)
{
  generate_marker(box_msgs->list);
}

void Visualize3DBox::generate_marker(const std::vector<object_msgs::ObjectInfo>& boxs)
{
  visualization_msgs::MarkerArray marker_array;
  for (auto box : boxs) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time::now();
    marker.type = marker.CUBE;
    marker.action = marker.ADD;
    marker.id = box.id;
    marker.pose.position.x = box.x;
    marker.pose.position.y = box.y;
    marker.pose.position.z = box.z;

    geometry_msgs::Quaternion q_pred = tf::createQuaternionMsgFromYaw(box.yaw);
    marker.pose.orientation.x = q_pred.x;
    marker.pose.orientation.y = q_pred.y;
    marker.pose.orientation.z = q_pred.z;
    marker.pose.orientation.w = q_pred.w;

    marker.lifetime = ros::Duration(0.125);
    marker.scale.x = box.length;
    marker.scale.y = box.width;
    marker.scale.z = box.hight;
    if (box.label == 2){
        marker.color.a = 0.5;
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 0;
    } else {
        marker.color.a = 0.5;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
    }
    marker_array.markers.push_back(marker);
  }
  marker_pub_.publish(marker_array);
}
