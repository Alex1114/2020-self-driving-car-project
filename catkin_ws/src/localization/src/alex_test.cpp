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
#include <pcl/registration/registration.h>
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
  Subscriber sub_lidar;
  Subscriber sub_fix;
//   Subscriber sub_imu;

  Publisher pub_voxel;
  Publisher pub_processed_pc;
  Publisher pub_map;
//   Publisher pub_fix_vis;
  Publisher marker_pub;

  tf::TransformListener listener;
  Eigen::Matrix4f eigenTrans;
// =================================
  Eigen::Vector3d fix_init;
  Eigen::Matrix4f initial_guess;
  Eigen::Matrix4f tf_icp;
  Eigen::Matrix3f transformation;
  IterativeClosestPoint<PointXYZ, PointXYZ> icp;
  PointCloud<PointXYZ>::Ptr result;

  
// =================================

  visualization_msgs::Marker marker;
  geometry_msgs::PointStamped fix;
  sensor_msgs::PointCloud2 ros_map;
  sensor_msgs::PointCloud2 ros_pc;
  

  
  
  sensor_msgs::PointCloud2 ros_processed_pc;
  sensor_msgs::PointCloud2 ros_cloud_msg;

  PointCloud<PointXYZ>::Ptr map;
  PointCloud<PointXYZ>::Ptr map_copy;

  PointCloud<PointXYZ>::Ptr lidar;
  PointCloud<PointXYZ>::Ptr processed_pc;

  VoxelGrid<PointXYZ> voxel;
  PassThrough<PointXYZ> passX;
  PassThrough<PointXYZ> passY;
  PassThrough<PointXYZ> passZ;
  StatisticalOutlierRemoval<PointXYZ> sor;

  float gcd = 100;
  float max_xy = 400;
  float max_z = 4;
  float leaf_size = 0.5;
  int count = 0;
// =====================
  int t = 0;
// =====================

//   void cb_all(const sensor_msgs::PointCloud2ConstPtr &pc, const geometry_msgs::PointStampedConstPtr &fix);
  void cb_lidar(const sensor_msgs::PointCloud2ConstPtr &pc);
  void cb_fix(const geometry_msgs::PointStamped fix);
//   void cb_imu(const sensor_msgs::Imu::ConstPtr&);


public:
  SimpleICP(NodeHandle nh);
  ~SimpleICP();
};

SimpleICP::SimpleICP(NodeHandle nh)
{
  //
  initial_guess  << -0.81915, -0.57357, 0, -263.5926208496094,
					0.57357, -0.81915, 0, -67.85790252685547,
					0, 0, 1, -9.890708923339844,
					0, 0, 0, 1;
  //
  // load map  -----------------------------------------
  char const *user = std::getenv("USER");
  stringstream ss;
  ss << "/home/" << user << "/2020-self-driving-car-project/data/ITRI/map.pcd";
  cout << "map: " << ss.str() << endl;

  map.reset(new PointCloud<PointXYZ>);
  map_copy.reset(new PointCloud<PointXYZ>);

  if (pcl::io::loadPCDFile<PointXYZ>(ss.str().c_str(), *map) == -1)
  {
	PCL_ERROR("Couldn't read file map.pcd \n");
	exit(0);
  }
  cout << "map size:" << map->size() << endl;
  // load map  -----------------------------------------------------------------------------------

  // point cloud init ----------------------------------------------------------------------------
  result.reset(new PointCloud<PointXYZ>());

  lidar.reset(new PointCloud<PointXYZ>);
  processed_pc.reset(new PointCloud<PointXYZ>);
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);

  passX.setFilterFieldName("x");
  passX.setFilterLimits(-max_xy, max_xy);

  passY.setFilterFieldName("y");
  passY.setFilterLimits(-max_xy, max_xy);

  passZ.setFilterFieldName("z");
  passZ.setFilterLimits(0, max_z);

  sor.setMeanK (50);
  sor.setStddevMulThresh (0.5);
  // point cloud init ----------------------------------------------------------------------------

  // vis init ------------------------------------------------------------------------------------
//   marker.header.frame_id = "map";
//   marker.scale.x = 0.5;
//   marker.color.r = 1;
//   marker.color.g = 0;
//   marker.color.b = 0;
//   marker.color.a = 1;
//   marker.type = visualization_msgs::Marker::LINE_STRIP;
//   marker.action = visualization_msgs::Marker::ADD;
  // vis init ------------------------------------------------------------------------------------

  // ros init ------------------------------------------------------------------------------------
  pub_voxel = nh.advertise<sensor_msgs::PointCloud2>("/lidar_voxel", 1);
  pub_processed_pc = nh.advertise<sensor_msgs::PointCloud2>("/lidar_visual", 1);
  pub_map = nh.advertise<sensor_msgs::PointCloud2>("/map", 1);
//   pub_fix_vis = nh.advertise<visualization_msgs::Marker>("/fix_vis", 1);


  sub_lidar = nh.subscribe("/lidar_points", 1, &SimpleICP::cb_lidar, this);
  sub_fix = nh.subscribe("/fix", 1, &SimpleICP::cb_fix, this);
//   sub_imu = nh.subscribe("/imu/data", 1, &SimpleICP::cb_imu, this);

  // ros init ------------------------------------------------------------------------------------

//   voxel.setInputCloud(map);
//   voxel.filter(*map);
  toROSMsg(*map, ros_map);
  ros_map.header.frame_id = "map";
  ros_map.header.stamp = ros::Time::now();
//   pub_map.publish(ros_map);
  cout << "pub map" << endl;

//

  marker.header.frame_id = "map";
//   marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

//
}

SimpleICP::~SimpleICP()
{
}


void SimpleICP::cb_lidar(const sensor_msgs::PointCloud2ConstPtr &pc)
{
  //   pcl_ros::transformPointCloud(eigenTrans, *pc, ros_pc);
  ros_pc = *pc;
//   fromROSMsg(ros_pc, *lidar);

//   voxel.setInputCloud(lidar);
//   voxel.filter(*lidar);
//   passX.setInputCloud(lidar);
//   passX.filter(*lidar);
//   passY.setInputCloud(lidar);
//   passY.filter(*lidar);
//   passZ.setInputCloud(lidar);
//   passZ.filter(*lidar);

//   toROSMsg(*lidar, ros_processed_pc);
//   ros_processed_pc.header.frame_id = "velodyne";
//   ros_processed_pc.header.stamp = ros::Time::now();
//   pub_processed_pc.publish(ros_processed_pc);
//   processed_pc->points.clear();

// =================================================
	// toROSMsg(*map, origin_map);
	// origin_map.header.frame_id = "map";
	// pc_map.publish(origin_map);

	fromROSMsg (ros_pc, *lidar);
	*map_copy = *map;
	// voxel.setInputCloud(lidar);
	// voxel.filter(*lidar);
	passX.setInputCloud(map_copy);
	passX.filter(*map_copy);
	passY.setInputCloud(map_copy);
	passY.filter(*map_copy);
	// passZ.setInputCloud(map_copy);
	// passZ.filter(*map_copy);
	// sor.setInputCloud (lidar);
	// sor.filter (*lidar);

	pcl::transformPointCloud (*lidar, *lidar, initial_guess);

	icp.setInputSource(lidar);
	icp.setInputTarget(map);
	icp.setMaxCorrespondenceDistance(100);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.001);
	icp.setMaximumIterations(100); 
	
	icp.align(*result);
	cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << endl;
	cout << icp.getFinalTransformation() << endl;
	tf_icp = icp.getFinalTransformation();

	cout << result->points.size() <<endl;

	initial_guess = tf_icp * initial_guess;

	toROSMsg(*lidar, ros_cloud_msg);
	ros_cloud_msg.header.frame_id = "map";
	pub_processed_pc.publish(ros_cloud_msg);

	geometry_msgs::Point p;
	p.x = initial_guess(0,3);
	p.y = initial_guess(1,3);
	p.z = initial_guess(2,3);;
	marker.points.push_back(p);
	marker.lifetime = ros::Duration();
	marker_pub.publish(marker);

	// tf::Vector3 tf_tran = tf::Vector3(initial_guess(0,3), initial_guess(1,3), initial_guess(2,3));
	// tf::Matrix3x3 tf_rot = tf::Matrix3x3(initial_guess(0,0), initial_guess(0,1), initial_guess(0,2),
	// 									initial_guess(1,0), initial_guess(1,1), initial_guess(1,2),
	// 									initial_guess(2,0), initial_guess(2,1), initial_guess(2,2));

	
	// tf::Transform tf = tf::Transform(tf_rot, tf_tran);
	// br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "map", "scan"));
// =================================================

}

void SimpleICP::cb_fix(const geometry_msgs::PointStamped fix)
{	

// ==============================================
  if (t == 0)
  {
	t = 1;
  	fix_init[0] = fix.point.x;
  	fix_init[1] = fix.point.y;
  	fix_init[2] = fix.point.z;
  	// cout << fix_init << endl;
	return;
  }
// ===============================================
//   marker.points.push_back(fix.point);
//   pub_fix_vis.publish(marker);
}

// void SimpleICP::cb_imu(const sensor_msgs::Imu::ConstPtr& msg){

//     Eigen::Quaternionf orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    
//     Eigen::Matrix3f Q = orientation.toRotationMatrix();
//     Q = transformation * Q;
//     Eigen::Quaternionf new_orientation(Q);
//     trans_data.orientation.w = new_orientation.w();
//     trans_data.orientation.x = new_orientation.x();
//     trans_data.orientation.y = new_orientation.y();
//     trans_data.orientation.z = new_orientation.z();
//     trans_data.header.stamp = msg->header.stamp;
// }


int main(int argc, char *argv[])
{
  init(argc, argv, "simpleICP");
  ROS_INFO("simpleICP init");

  NodeHandle nh;
  SimpleICP voxel(nh);
  spin();
  return 0;
}
