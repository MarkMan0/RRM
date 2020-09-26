#include "rrm_cv1_pal/turtle_control.h"

int main(int argc, char** argv)
{
  // Init node
  ros::init(argc, argv, "turtle_control");

  // Create class instance
  TurtleControl turtleControl;

  // Set loop rate (hz)
  ros::Rate loop_rate(turtleControl.getRate());

  // Control turtle in loop
  while (ros::ok())
  {
    turtleControl.tick();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
