#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int16MultiArray.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>

// Pcl load and ros
#include <pcl/PolygonMesh.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

using namespace ros;
using namespace pcl;
using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PointStamped>
    MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

class SimpleICP
{
private:
  boost::shared_ptr<Sync> sync_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_lidar;
  message_filters::Subscriber<geometry_msgs::PointStamped> sub_fix;
  message_filters::Subscriber<sensor_msgs::Imu> sub_imu;

  Publisher pub_voxel;
  Publisher pub_processed_pc;
  Publisher pub_map;
  Publisher pub_fix_vis;

  tf::TransformListener listener;
  Eigen::Matrix4f eigenTrans;

  visualization_msgs::Marker marker;
  geometry_msgs::PointStamped fix;
  sensor_msgs::PointCloud2 ros_map;
  sensor_msgs::PointCloud2 ros_pc;
  sensor_msgs::PointCloud2 ros_processed_pc;

  PointCloud<PointXYZ>::Ptr map;
  PointCloud<PointXYZ>::Ptr lidar;
  PointCloud<PointXYZ>::Ptr processed_pc;

  VoxelGrid<PointXYZ> voxel;
  PassThrough<PointXYZ> passX;
  PassThrough<PointXYZ> passY;
  PassThrough<PointXYZ> passZ;

  float gcd = 100;
  float max_xy = 6;
  float max_z = 4;
  float leaf_size = 1;
  int count = 0;
  void cb_all(const sensor_msgs::PointCloud2ConstPtr &pc, const geometry_msgs::PointStampedConstPtr &fix);
  void cb_lidar(const sensor_msgs::PointCloud2ConstPtr &pc);
  void cb_fix(const geometry_msgs::PointStamped fix);

public:
  SimpleICP(NodeHandle nh);
  ~SimpleICP();
};

SimpleICP::SimpleICP(NodeHandle nh)
{
  //   tf::StampedTransform transform;
  //   try
  //   {
  //     ros::Duration five_seconds(5.0);
  //     listener.waitForTransform("X1/base_footprint", "X1/front_laser", ros::Time(0), five_seconds);
  //     listener.lookupTransform("X1/base_footprint", "X1/front_laser", ros::Time(0), transform);
  //   }
  //   catch (tf::TransformException ex)
  //   {
  //     ROS_ERROR("%s", ex.what());
  //     return;
  //   }
  //   pcl_ros::transformAsMatrix(transform, eigenTrans);

  // load map  -----------------------------------------
  char const *user = std::getenv("USER");
  stringstream ss;
  ss << "/home/" << user << "/2020-self-driving-car-project/data/ITRI/map.pcd";
  cout << "map: " << ss.str() << endl;

  map.reset(new PointCloud<PointXYZ>);
  if (pcl::io::loadPCDFile<PointXYZ>(ss.str().c_str(), *map) == -1)
  {
    PCL_ERROR("Couldn't read file map.pcd \n");
    exit(0);
  }
  cout << "map size:" << map->size() << endl;
  // load map  -----------------------------------------------------------------------------------

  // point cloud init ----------------------------------------------------------------------------
  lidar.reset(new PointCloud<PointXYZ>);
  processed_pc.reset(new PointCloud<PointXYZ>);
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);

  passX.setFilterFieldName("x");
  passX.setFilterLimits(-max_xy, max_xy);

  passY.setFilterFieldName("y");
  passY.setFilterLimits(-max_xy, max_xy);

  passZ.setFilterFieldName("z");
  passZ.setFilterLimits(0, max_z);
  // point cloud init ----------------------------------------------------------------------------

  // vis init ------------------------------------------------------------------------------------
  marker.header.frame_id = "icp_map";
  marker.scale.x = 0.5;
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 1;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  // vis init ------------------------------------------------------------------------------------

  // ros init ------------------------------------------------------------------------------------
  pub_voxel = nh.advertise<sensor_msgs::PointCloud2>("/lidar_voxel", 1);
  pub_processed_pc = nh.advertise<sensor_msgs::PointCloud2>("/lidar_visual", 1);
  pub_map = nh.advertise<sensor_msgs::PointCloud2>("/map", 1);
  pub_fix_vis = nh.advertise<visualization_msgs::Marker>("/fix_vis", 1);

  sub_lidar.subscribe(nh, "/lidar_points", 1);
  sub_fix.subscribe(nh, "/fix", 1);

  sync_.reset(new Sync(MySyncPolicy(1), sub_lidar, sub_fix));
  sync_->registerCallback(boost::bind(&SimpleICP::cb_all, this, _1, _2));
  // ros init ------------------------------------------------------------------------------------

  voxel.setInputCloud(map);
  voxel.filter(*map);
  toROSMsg(*map, ros_map);
  ros_map.header.frame_id = "icp_map";
  ros_map.header.stamp = ros::Time::now();
  pub_map.publish(ros_map);
  cout << "pub map" << endl;
}

SimpleICP::~SimpleICP()
{
}

void SimpleICP::cb_all(const sensor_msgs::PointCloud2ConstPtr &pc, const geometry_msgs::PointStampedConstPtr &fix)
{
  count++;
  cout << count << ", ";
  cout << pc->header.stamp << ", ";
  cout << fix->header.stamp << ", ";
  cout << endl;
}

void SimpleICP::cb_lidar(const sensor_msgs::PointCloud2ConstPtr &pc)
{
  //   pcl_ros::transformPointCloud(eigenTrans, *pc, ros_pc);
  ros_pc = *pc;
  fromROSMsg(ros_pc, *lidar);

  //   voxel.setInputCloud(lidar);
  //   voxel.filter(*lidar);
  //   passX.setInputCloud(lidar);
  //   passX.filter(*lidar);
  //   passY.setInputCloud(lidar);
  //   passY.filter(*lidar);
  //   passZ.setInputCloud(lidar);
  //   passZ.filter(*lidar);

  toROSMsg(*lidar, ros_processed_pc);
  ros_processed_pc.header.frame_id = "velodyne";
  ros_processed_pc.header.stamp = ros::Time::now();
  pub_processed_pc.publish(ros_processed_pc);
  processed_pc->points.clear();
}

void SimpleICP::cb_fix(const geometry_msgs::PointStamped fix)
{
  marker.points.push_back(fix.point);
  pub_fix_vis.publish(marker);
}

int main(int argc, char *argv[])
{
  init(argc, argv, "simpleICP");
  ROS_INFO("simpleICP init");

  NodeHandle nh;
  SimpleICP voxel(nh);
  spin();
  return 0;
}
