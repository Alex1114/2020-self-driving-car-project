// ros
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
// pcl
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

class ICP_method1
{
private:
    Publisher pub_map;
    Publisher pub_lidar;
    Publisher pub_result;
    Subscriber sub_lidar;
    Subscriber sub_fix;
    tf::TransformListener listener;

    VoxelGrid<PointXYZ> voxel;
    PassThrough<PointXYZ> passX;
    PassThrough<PointXYZ> passY;
    PassThrough<PointXYZ> passZ;
    StatisticalOutlierRemoval<PointXYZ> sor;

    PointCloud<PointXYZ>::Ptr map;
    PointCloud<PointXYZ>::Ptr map_copy;
    PointCloud<PointXYZ>::Ptr lidar;
    PointCloud<PointXYZ>::Ptr result;

    sensor_msgs::PointCloud2 ros_map_copy;
    sensor_msgs::PointCloud2 ros_lidar;

    bool init_guess = false;
    Eigen::Matrix4f initial_guess;
    Eigen::Matrix4f tf_icp;
    Eigen::Matrix4f v2b;

    int frame = 1;
    float max_xy = 50;
    float max_car_xx = 30;
    float max_car_x = 30;
    float max_car_yy = 15;
    float max_car_y = 15;
    float max_car_z = 10;
    float min_car_z = -10;
    float leaf_size = 0.5;
    double r=0, p=0, y=2.97;

public:
    ICP_method1(NodeHandle &);
    void cb_lidar(const sensor_msgs::PointCloud2ConstPtr &pc);
    void cb_fix(const geometry_msgs::PointStamped fix);
};

ICP_method1::ICP_method1(NodeHandle &nh)
{
    // parameter
    // -----------------------------------------------------------------------------------
    param::get("~max_xy", max_xy);
    param::get("~max_car_xx", max_car_xx);
    param::get("~max_car_x", max_car_x);
    param::get("~max_car_yy", max_car_yy);
    param::get("~max_car_y", max_car_y);
    param::get("~max_car_z", max_car_z);
    param::get("~min_car_z", min_car_z);
    param::get("~yaw", y);
    ROS_INFO("max_xy %f", max_xy);
    ROS_INFO("max_car_xx %f", max_car_xx);
    ROS_INFO("max_car_x %f", max_car_x);
    ROS_INFO("max_car_yy %f", max_car_yy);
    ROS_INFO("max_car_y %f", max_car_y);
    ROS_INFO("max_car_z %f", max_car_z);
    ROS_INFO("min_car_z %f", min_car_z);
    ROS_INFO("yaw %f", y);

    // -----------------------------------------------------------------------------------

    // load map
    // -----------------------------------------------------------------------------------
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
    cout << "---------------------------" << endl;
    // -----------------------------------------------------------------------------------

    // point cloud init
    // ----------------------------------------------------------------------------
    lidar.reset(new PointCloud<PointXYZ>);
    result.reset(new PointCloud<PointXYZ>());

    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    passX.setFilterFieldName("x");
    passY.setFilterFieldName("y");
    passZ.setFilterFieldName("z");
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.5);
    // ----------------------------------------------------------------------------

    // subscribe & pub
    // -----------------------------------------------------------------------------------
    pub_map = nh.advertise<sensor_msgs::PointCloud2>("/map_crop", 1);
    pub_result = nh.advertise<geometry_msgs::PoseStamped>("/result", 1);
    pub_lidar = nh.advertise<sensor_msgs::PointCloud2>("/lidar_mapped", 1);
    sub_lidar = nh.subscribe("/lidar_points", 0, &ICP_method1::cb_lidar, this);
    // -----------------------------------------------------------------------------------
}

void ICP_method1::cb_lidar(const sensor_msgs::PointCloud2ConstPtr &pc)
{
    // initial_guess
    // -------------------------------------------------------------------------------
    if (!init_guess)
    {
        // sensor_msgs::Imu::ConstPtr imu = topic::waitForMessage<sensor_msgs::Imu>("/imu/data", Duration(1));
        // Eigen::Quaternionf q(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
        // Eigen::Matrix3f mat = q.toRotationMatrix();

        tf::Quaternion q_rot;
        q_rot.setRPY(r, p, y);
        tf::Matrix3x3 mat;
        mat.setRotation(q_rot);
 
        geometry_msgs::PointStamped::ConstPtr gps = topic::waitForMessage<geometry_msgs::PointStamped>("/fix", Duration(1));
        initial_guess << mat[0][0], mat[0][1], mat[0][2], gps->point.x + 2,
            mat[1][0], mat[1][1], mat[1][2], gps->point.y - 2,
            mat[2][0], mat[2][1], mat[2][2], gps->point.z,
            0, 0, 0, 1;
        // initial_guess << mat(0,0), mat(0,1), mat(0,2), gps->point.x,
        //     mat(1,0), mat(1,1), mat(1,2), gps->point.y,
        //     mat(2,0), mat(2,1), mat(2,2), gps->point.z,
        //     0, 0, 0, 1;
  
        cout << "initial guess :" << endl;
        cout << initial_guess << endl;
        cout << "---------------------------" << endl;
        init_guess = true;
    }
    // -------------------------------------------------------------------------------

    fromROSMsg(*pc, *lidar);
    *map_copy = *map;
    // position
    // ------------------------------------------------------------------------------------
    geometry_msgs::Point p;
    p.x = initial_guess(0, 3);
    p.y = initial_guess(1, 3);
    p.z = initial_guess(2, 3);
    // cout << "x:" << p.x << endl;
    // cout << "y:" << p.y << endl;
    // cout << "z:" << p.z << endl;
    // cout << endl;
    // ------------------------------------------------------------------------------------

    // map_filter
    // ----------------------------------------------------------------------------------------
    passX.setFilterLimits(p.x - max_xy, p.x + max_xy);
    passX.setInputCloud(map_copy);
    passX.filter(*map_copy);

    passY.setFilterLimits(p.y - max_xy, p.y + max_xy);
    passY.setInputCloud(map_copy);
    passY.filter(*map_copy);

    // passZ.setFilterLimits(-12, -5);
    // passZ.setInputCloud(map_copy);
    // passZ.filter(*map_copy);

    // voxel.setInputCloud(map_copy);
    // voxel.filter(*map_copy);

    // sor.setInputCloud(map_copy);
    // sor.filter(*map_copy);

    cout << "map_copy size:" << map_copy->points.size() << endl;
    toROSMsg(*map_copy, ros_map_copy);
    ros_map_copy.header.frame_id = "mapped";
    ros_map_copy.header.stamp = pc->header.stamp;
    pub_map.publish(ros_map_copy);
    // ----------------------------------------------------------------------------------------

    // transform
    // -----------------------------------------------------------------------------------
    tf::StampedTransform tf_v2b;
    try
    {
        ros::Duration five_seconds(5.0);
        listener.waitForTransform("/velodyne", "/base_link", ros::Time(0), five_seconds);
        listener.lookupTransform("/velodyne", "/base_link", ros::Time(0), tf_v2b);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    pcl_ros::transformAsMatrix(tf_v2b, v2b);
    // ----------------------------------------------------------------------------------------

    // lidar_filter
    // ----------------------------------------------------------------------------------------
    passX.setFilterLimits(-max_car_x, max_car_xx);
    passX.setInputCloud(lidar);
    passX.filter(*lidar);

    passY.setFilterLimits(-max_car_y, max_car_yy);
    passY.setInputCloud(lidar);
    passY.filter(*lidar);

    passZ.setFilterLimits(min_car_z + v2b(2, 3), max_car_z + v2b(2, 3));
    passZ.setInputCloud(lidar);
    passZ.filter(*lidar);

    // voxel.setInputCloud(lidar);
    // voxel.filter(*lidar);

    // sor.setInputCloud(lidar);
    // sor.filter(*lidar);
    // --------------------------------------------------------------------------------------

    // icp
    // -----------------------------------------------------------------------------------------
    IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    pcl::transformPointCloud(*lidar, *lidar, initial_guess);
    icp.setInputSource(lidar);
    icp.setInputTarget(map_copy);
    icp.setMaxCorrespondenceDistance(1000);
    icp.setTransformationEpsilon(1e-12);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setMaximumIterations(2000);
    // icp.setRANSACOutlierRejectionThreshold(1.5);
    icp.align(*result);

    tf_icp = icp.getFinalTransformation();
    cout << "frame:" << frame << endl;
    cout << "result point size:" << result->points.size() << endl;
    cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;
    // --------------------------------------------------------------------------------------------

    // result
    // --------------------------------------------------------------------------------------------
    Eigen::Matrix4f tf_result;
    tf_result = tf_icp * initial_guess * v2b;

    Eigen::Matrix3f rotation;
    rotation << tf_result(0, 0), tf_result(0, 1), tf_result(0, 2),
        tf_result(1, 0), tf_result(1, 1), tf_result(1, 2),
        tf_result(2, 0), tf_result(2, 1), tf_result(2, 2);
    Eigen::Quaternionf quat(rotation);

    geometry_msgs::PoseStamped result_pose;
    result_pose.header.stamp = pc->header.stamp;
    result_pose.pose.position.x = tf_result(0, 3);
    result_pose.pose.position.y = tf_result(1, 3);
    result_pose.pose.position.z = tf_result(2, 3);
    result_pose.pose.orientation.x = quat.x();
    result_pose.pose.orientation.y = quat.y();
    result_pose.pose.orientation.z = quat.z();
    result_pose.pose.orientation.w = quat.w();
    pub_result.publish(result_pose);
    // -----------------------------------------------------------------------------------------

    // publish
    // -------------------------------------------------------------------------------------
    toROSMsg(*result, ros_lidar);
    ros_lidar.header.frame_id = "mapped";
    ros_lidar.header.stamp = pc->header.stamp;
    pub_lidar.publish(ros_lidar);
    // -------------------------------------------------------------------------------------

    //  reset
    // -------------------------------------------------------------------------------------
    map_copy->points.clear();
    lidar->points.clear();
    initial_guess = tf_icp * initial_guess;
    cout << "--------------------------------" << endl;
    // -------------------------------------------------------------------------------------
    frame += 1;
}


int main(int argc, char *argv[])
{
    init(argc, argv, "ICP_method1");
    ROS_INFO("ICP_method1 init");
    NodeHandle nh;
    ICP_method1 ICP_method1(nh);
    spin();
    return 0;
}
