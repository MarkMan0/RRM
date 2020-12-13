#include "laser_to_pcl_pkg/laser_to_pcl.h"

LaserScanToPointCloud::LaserScanToPointCloud()
  : laser_sub_(n_, "/abb_irb/laser/scan", 20), laser_notifier_(laser_sub_, listener_, "tool0", 20)
{
  ROS_INFO("Initializing scan listener");
  // init point cloud
  cloud_concat_.header.stamp = ros::Time::now();
  cloud_concat_.header.frame_id = "tool0";
  // init laser scan transformation
  laser_notifier_.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
  laser_notifier_.setTolerance(ros::Duration(TRANSFORM_TIME_TOLERANCE_));
  scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/abb_irb/laser/cloud", 1);
}

void LaserScanToPointCloud::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  // project laser scan to 3D pointcloud
  try
  {
    projector_.transformLaserScanToPointCloud("world", *scan_msg, cloud_reading_, listener_);
    if (first_scan_)
    {
      first_scan_ = false;
      cloud_concat_ = cloud_reading_;
    }
  }
  catch (tf::TransformException& e)
  {
    std::cout << e.what();
    return;
  }  // Append new scan to cloud
  pcl::concatenatePointCloud(cloud_concat_, cloud_reading_, cloud_concat_);
  // publish scanned object in scan_object frame
  cloud_concat_.header.stamp = ros::Time::now();
  scan_pub_.publish(cloud_concat_);
}
