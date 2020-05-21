#include <iostream>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace ros;

class Drawer
{
private:
  visualization_msgs::Marker marker;
  Publisher pub_marker;
  Subscriber sub_odom;

public:
  Drawer(NodeHandle nh);
  void cb_odom(const geometry_msgs::PoseWithCovarianceStamped &p);
  ~Drawer();
};

Drawer::Drawer(NodeHandle nh)
{
  ROS_INFO("init Drawer");

  marker.scale.x = 0.2;
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 1;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  pub_marker = nh.advertise<visualization_msgs::Marker>("ekf_marker", 1);
  sub_odom = nh.subscribe("/robot_pose_ekf/odom_combined", 1, &Drawer::cb_odom, this);
}

void Drawer::cb_odom(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
  marker.header.frame_id = msg.header.frame_id;
  marker.header.stamp = msg.header.stamp;

  geometry_msgs::Point p;
  p.x = msg.pose.pose.position.x;
  p.y = msg.pose.pose.position.y;
  p.z = msg.pose.pose.position.z;
  marker.points.push_back(p);

  pub_marker.publish(marker);
}

Drawer::~Drawer()
{
}

int main(int argc, char *argv[])
{
  init(argc, argv, "Drawer");
  NodeHandle nh;
  Drawer cov(nh);
  spin();
  return 0;
}
