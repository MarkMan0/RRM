#ifndef PROJECT_TURTLECONTROL_H
#define PROJECT_TURTLECONTROL_H

// Include ros base
#include <ros/ros.h>

// Include ros msgs
#include <geometry_msgs/Twist.h>
#include <rrm_cv1_pal/Draw.h>
#include <rrm_cv1_pal/Draw_Circle.h>
#include <turtlesim/Pose.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>

// main class for turtle control
class TurtleControl
{
public:
  // Constructor
  TurtleControl();

  // Callbacks
  bool drawCallback(rrm_cv1_pal::Draw::Request& req, rrm_cv1_pal::Draw::Response& res);
  bool drawCircleCallback(rrm_cv1_pal::Draw_Circle::Request&, rrm_cv1_pal::Draw_Circle::Response&);

  void poseCallback(const turtlesim::Pose::ConstPtr& msg);

  // Other public methods
  void setupDrawing();
  void publish();
  double getRate();
  bool getDrawingStatus();

private:
  // Consts
  const double PUB_RATE = 100.0;
  const double WINDOW_CENTER = 5.544444561;
  const double WINDOW_EDGE = 11.088889122;

  void resetVelocity();

  // member variables
  geometry_msgs::Twist velocity_msg_;
  bool drawing_status_;
  turtlesim::Pose pose_msg_;

  // ROS communication tools
  ros::Publisher velocity_pub_;
  ros::Subscriber pose_sub_;
  ros::ServiceServer square_service_, draw_circle_service_;
  ros::ServiceClient teleport_client_;
  ros::ServiceClient clear_client;
};

#endif  // PROJECT_TURTLECONTROL_H