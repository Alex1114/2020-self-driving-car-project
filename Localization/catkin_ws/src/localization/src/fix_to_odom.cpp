#include <iostream>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

using namespace std;
using namespace ros;

class Fix2Odom
{
private:
  nav_msgs::Odometry gps;

  Publisher pub_odom;
  Subscriber sub_fix;

public:
  Fix2Odom(NodeHandle nh);
  void cb_fix(const geometry_msgs::PointStamped &p);
  ~Fix2Odom();
};

Fix2Odom::Fix2Odom(NodeHandle nh)
{
  ROS_INFO("init fix2odom");

  gps.header.frame_id = "base_link";

  pub_odom = nh.advertise<nav_msgs::Odometry>("gps", 1);
  sub_fix = nh.subscribe("fix", 1, &Fix2Odom::cb_fix, this);
}

void Fix2Odom::cb_fix(const geometry_msgs::PointStamped &p)
{
  float cov = 0.001;
  gps.header.stamp = p.header.stamp;
  gps.pose.pose.position.x = p.point.x;
  gps.pose.pose.position.y = p.point.y;
  gps.pose.pose.position.z = p.point.z;
  gps.pose.pose.orientation.x = 0;
  gps.pose.pose.orientation.y = 0;
  gps.pose.pose.orientation.z = 0;
  gps.pose.pose.orientation.w = 1;
  gps.pose.covariance = { cov, 0, 0, 0, 0, 0,
                          0, cov, 0, 0, 0, 0, 
                          0, 0, cov, 0, 0, 0,
                          0, 0, 0, 99999, 0, 0, 
                          0, 0, 0, 0, 99999, 0, 
                          0, 0, 0, 0, 0, 99999 };
  pub_odom.publish(gps);
}

Fix2Odom::~Fix2Odom()
{
}

int main(int argc, char *argv[])
{
  init(argc, argv, "fix2odom");
  NodeHandle nh;
  Fix2Odom cov(nh);
  spin();
  return 0;
}
