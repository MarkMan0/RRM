#include "rrm_cv3_pal/forward_kinematics.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_tf_broadcaster");

  ForwardKinematics kinematics;
  ros::Rate rate(100);

  while (ros::ok())
  {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    kinematics.broadcastTf();
    rate.sleep();
  }

  return 0;
}
