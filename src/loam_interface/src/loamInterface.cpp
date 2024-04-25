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


using namespace std;

const double PI = 3.1415926;

class InterfaceHandler
{
  public:
  ros::NodeHandle &nh_;
  // Create a buffer for managing transforms
  tf::TransformListener listener_;

  nav_msgs::Odometry odomData;
  tf::StampedTransform odomTrans;


  ros::Subscriber odom_sub_;
  ros::Subscriber laser_sub_;
  ros::Publisher odometry_pub_;
  ros::Publisher pcl_pub_;

  tf::TransformBroadcaster tfBroadcaster;

  bool flipStateEstimation = false;
  bool flipRegisteredScan = false;
  bool sendTF = true;
  bool reverseTF = false;

  InterfaceHandler(ros::NodeHandle &nh): nh_(nh)
  {
    odom_sub_ = nh.subscribe("/turtlebot/kobuki/odom_ground_truth", 1, &InterfaceHandler::odometryCallback, this);

    laser_sub_ = nh.subscribe("/cloud_registered_scan", 1, &InterfaceHandler::laserCloudCallback, this);

    odometry_pub_ = nh.advertise<nav_msgs::Odometry> ("/state_estimation", 5);

    pcl_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("/registered_scan", 5);
  };

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom)
  {
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
    odomData = *odom;

    if (flipStateEstimation) {
      tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

      pitch = -pitch;
      yaw = -yaw;

      geoQuat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

      odomData.pose.pose.orientation = geoQuat;
      odomData.pose.pose.position.x = odom->pose.pose.position.z;
      odomData.pose.pose.position.y = odom->pose.pose.position.x;
      odomData.pose.pose.position.z = odom->pose.pose.position.y;
    }

    // publish odometry messages
    odomData.header.frame_id = "map";
    odomData.child_frame_id = "sensor";
    odometry_pub_.publish(odomData);

    // publish tf messages
    odomTrans.stamp_ = odom->header.stamp;
    odomTrans.frame_id_ = "map";
    odomTrans.child_frame_id_ = "sensor";
    odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    odomTrans.setOrigin(tf::Vector3(odomData.pose.pose.position.x, odomData.pose.pose.position.y, odomData.pose.pose.position.z));

    if (sendTF) {
      if (!reverseTF) {
        tfBroadcaster.sendTransform(odomTrans);
      } else {
        tfBroadcaster.sendTransform(tf::StampedTransform(odomTrans.inverse(), odom->header.stamp, "sensor", "map"));
      }
    }
  }

  void laserCloudCallback(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudOut(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloud->clear();
    pcl::fromROSMsg(*laserCloudIn, *laserCloud);

    // Saving the current key frame of the robot
    std::string targetFrame = "map";
    std::string sourceFrame = "turtlebot/kobuki/realsense_depth";

    // Lookup transform
    tf::StampedTransform transform;
    listener_.lookupTransform(targetFrame, sourceFrame, ros::Time(), transform);

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
    laserCloud2.header.stamp = laserCloudIn->header.stamp;
    laserCloud2.header.frame_id = targetFrame;
    pcl_pub_.publish(laserCloud2);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "loamInterface");
  ros::NodeHandle nh;

  InterfaceHandler handler(nh);

  ros::spin();

  return 0;
}
