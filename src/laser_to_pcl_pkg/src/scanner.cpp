#include "ros/ros.h"
#include "laser_to_pcl_pkg/laser_to_pcl.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "l2p");
  ros::NodeHandle node();
  LaserScanToPointCloud l_to_p;
  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}