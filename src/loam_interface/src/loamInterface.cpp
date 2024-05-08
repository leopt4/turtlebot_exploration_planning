/**
 * @file sensor_coverage_planner_ground.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that does the job of exploration
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "loam_interface/loamInterFace.h"

InterfaceHandler::InterfaceHandler(ros::NodeHandle &nh, ros::NodeHandle &nh_p)
  {
    readParameters(nh_p);
    // odom_sub_ = nh.subscribe("/turtlebot/kobuki/odom_ground_truth", 1, &InterfaceHandler::odometryCallback, this);
    while (!listener_.canTransform(map_frame_, depth_camera_frame_, ros::Time(0))) {
      ros::Duration(0.1).sleep();
      ROS_INFO("Waiting for transform between map frame and depth camera frame...");
    }
    ROS_INFO("Init Loam Interface successfully.");

    laser_sub_ = nh.subscribe(sub_cloud_registered_scan_topic_, 1, &InterfaceHandler::laserCloudCallback, this);

    // odometry_pub_ = nh.advertise<nav_msgs::Odometry> ("/state_estimation", 5);

    pcl_pub_ = nh.advertise<sensor_msgs::PointCloud2> (sub_registered_scan_topic_, 5);

  };

bool InterfaceHandler::readParameters(ros::NodeHandle& nh)
  {
    map_frame_ = misc_utils_ns::getParam<std::string>(nh, "map_frame_", "map");
    depth_camera_frame_ = misc_utils_ns::getParam<std::string>(nh, "depth_camera_frame_", "turtlebot/kobuki/realsense_depth");

    sub_cloud_registered_scan_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_cloud_registered_scan_topic_", "/cloud_registered_scan");

    sub_registered_scan_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_registered_scan_topic_", "/registered_scan");
  }

void InterfaceHandler::odometryCallback(const nav_msgs::Odometry::ConstPtr& odom)
  {
    odomData = *odom;

    // publish odometry messages
    odomData.header.frame_id = "map";
    odomData.child_frame_id = "sensor";
    odometry_pub_.publish(odomData);

    // publish tf sensor messages
    sensorTrans.stamp_ = odom->header.stamp;
    // sensorTrans.frame_id_ = "turtlebot/kobuki/realsense_depth";
    sensorTrans.frame_id_ = "realsense_depth_frame";
    sensorTrans.child_frame_id_ = "sensor";
    sensorTrans.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    sensorTrans.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

    tfBroadcaster.sendTransform(sensorTrans);

    // publish tf map messages
    mapTrans.stamp_ = odom->header.stamp;
    mapTrans.frame_id_ = "turtlebot/kobuki/base_footprint";
    mapTrans.child_frame_id_ = "map";

    odomData.pose.pose.position.x = 0.0;
    odomData.pose.pose.position.y = 0.0;
    odomData.pose.pose.position.z = 0.0;
    odomData.pose.pose.orientation.x = 0.0;
    odomData.pose.pose.orientation.y = 0.0;
    odomData.pose.pose.orientation.z = 0.0;
    odomData.pose.pose.orientation.w = 1.0;
    // Create transform from odometry frame to base frame
    tf::Transform transformBaseToOdometry;
    transformBaseToOdometry.setOrigin(tf::Vector3(odomData.pose.pose.position.x,odomData.pose.pose.position.y, odomData.pose.pose.position.z));
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(odomData.pose.pose.orientation, orientation);
    transformBaseToOdometry.setRotation(orientation);
    
    // Invert the transform if needed
    tf::Transform transformOdometryToBase = transformBaseToOdometry.inverse();

    mapTrans.setRotation(transformOdometryToBase.getRotation());
    mapTrans.setOrigin(transformOdometryToBase.getOrigin());
    
    tfBroadcaster.sendTransform(mapTrans);

    frameAInitialized = true;
  }

void InterfaceHandler::laserCloudCallback(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
  {
    if (frameAInitialized)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudOut(new pcl::PointCloud<pcl::PointXYZ>());
      laserCloud->clear();
      pcl::fromROSMsg(*laserCloudIn, *laserCloud);
      
      // Lookup transform
      tf::StampedTransform transform;

      listener_.lookupTransform(map_frame_, depth_camera_frame_, ros::Time(), transform);

      pcl_ros::transformPointCloud(*laserCloud, *laserCloudOut, transform);

      if (flipRegisteredScan) {
        int laserCloudSize = laserCloudOut->points.size();
        for (int i = 0; i < laserCloudSize; i++) {
          float temp = laserCloud->points[i].x;
          laserCloudOut->points[i].x = laserCloudOut->points[i].z;
          laserCloudOut->points[i].z = laserCloudOut->points[i].y;
          laserCloudOut->points[i].y = temp;
        }
      }

      // publish registered scan messages
      sensor_msgs::PointCloud2 laserCloud2;
      pcl::toROSMsg(*laserCloudOut, laserCloud2);
      laserCloud2.header.stamp    = laserCloudIn->header.stamp;
      laserCloud2.header.frame_id = map_frame_;
      pcl_pub_.publish(laserCloud2);
    }
  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "loamInterface");
  ros::NodeHandle nh;
  ros::NodeHandle nh_p("~");

  InterfaceHandler loaminterface(nh, nh_p);

  ros::spin();

  return 0;
}
