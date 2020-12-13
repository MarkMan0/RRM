#ifndef LASERSCANTOPOINTCLOUD_H
#define LASERSCANTOPOINTCLOUD_H

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
class LaserScanToPointCloud
{
public:
  // Constructor
  LaserScanToPointCloud();
  // callbacks
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr&);

private:
  // ros node handle
  ros::NodeHandle n_;
  // laser scan transformation
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  bool first_scan_ = true;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  // publishers
  ros::Publisher scan_pub_;  // scanned object cloud messages
  sensor_msgs::PointCloud2 cloud_concat_;
  // temporary cloud for transformation
  sensor_msgs::PointCloud2 cloud_reading_;

  static constexpr double TRANSFORM_TIME_TOLERANCE_ = 0.5;
};
#endif  // LASERSCANTOPOINTCLOUD_H