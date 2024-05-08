/**
 * @file sensor_coverage_planner_ground.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that does the job of exploration
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h> // Include tf header
#include <tf2_ros/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/transforms.h>

#include <utils/misc_utils.h>

using namespace std;

class InterfaceHandler
{
    public:
    // Create a buffer for managing transforms
    tf::TransformListener listener_;

    nav_msgs::Odometry odomData;
    tf::StampedTransform sensorTrans;
    tf::StampedTransform mapTrans;


    ros::Subscriber odom_sub_;
    ros::Subscriber laser_sub_;
    ros::Publisher odometry_pub_;
    ros::Publisher pcl_pub_;

    tf::TransformBroadcaster tfBroadcaster;

    std::string map_frame_;
    std::string depth_camera_frame_;
    std::string sub_registered_scan_topic_;
    std::string sub_cloud_registered_scan_topic_;

    bool frameAInitialized  = true;
    bool flipRegisteredScan = false;

    explicit InterfaceHandler(ros::NodeHandle &nh, ros::NodeHandle &nh_p);
    bool readParameters(ros::NodeHandle& nh);


    void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom);
    void laserCloudCallback(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn);

}; // namespace loam_interface
