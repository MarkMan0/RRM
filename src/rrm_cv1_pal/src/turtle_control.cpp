#include "rrm_cv1_pal/turtle_control.h"
#include "std_srvs/Empty.h"

TurtleControl::TurtleControl()
{
  // NodeHandler
  ros::NodeHandle n;

  // Publisher
  velocity_pub_ = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

  // Subscriber
  pose_sub_ = n.subscribe("turtle1/pose", 1, &TurtleControl::poseCallback, this);

  // Service server
  square_service_ = n.advertiseService("/turtle_control/draw", &TurtleControl::drawCallback, this);
  draw_circle_service_ = n.advertiseService("/turtle_control/draw_circle", &TurtleControl::drawCircleCallback, this);

  // Service client
  teleport_client_ = n.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");

  clear_client = n.serviceClient<std_srvs::Empty>("/clear");

  ros::ServiceClient setpen_client = n.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");

  // init variables
  velocity_msg_ = geometry_msgs::Twist();

  // set default pen width
  int width = 1;

  // get parameters from server
  if (!n.getParam("/turtle_control/line/width", width))
  {
    ROS_WARN("Failed to get parameter /turtle_control/line/width, using 1");
    width = 1;
  }

  std::vector<int> color;
  if (!n.getParam("/turtle_control/line/colorRGB", color) && color.size() != 3)
  {
    ROS_WARN("Failed to get parameter /turtle_control/line/colorRGB, using [255, 255, 255]");
    color.clear();
    color.push_back(255);
    color.push_back(255);
    color.push_back(255);
  }

  // Create service message
  turtlesim::SetPen setpen_srv;
  setpen_srv.request.width = width;
  setpen_srv.request.off = false;
  setpen_srv.request.r = color[0];
  setpen_srv.request.g = color[1];
  setpen_srv.request.b = color[2];

  // Call service
  setpen_client.waitForExistence(ros::Duration(2));
  if (!setpen_client.call(setpen_srv))
  {
    ROS_WARN("Couldn't call setpen client");
  }

  // variable init
  this->drawing_status_ = false;
  this->pose_msg_ = turtlesim::Pose();
}

void TurtleControl::resetVelocity()
{
  velocity_msg_ = geometry_msgs::Twist();
}

// service server callback for starting the drawing and drawing speed configuration
bool TurtleControl::drawCallback(rrm_cv1_pal::Draw::Request& req, rrm_cv1_pal::Draw::Response& res)
{
  resetVelocity();
  velocity_msg_.linear.x = req.speed;
  this->drawing_status_ = true;
  res.success = true;
  return true;
}

bool TurtleControl::drawCircleCallback(rrm_cv1_pal::Draw_Circle::Request& req, rrm_cv1_pal::Draw_Circle::Response& res)
{
  resetVelocity();
  if (req.speed == 0 || req.radius == 0)
  {
    drawing_status_ = false;
    res.sucess = false;
  }
  else
  {
    velocity_msg_.linear.x = req.speed;
    velocity_msg_.angular.z = static_cast<double>(req.speed) / req.radius;
    drawing_status_ = true;
    res.sucess = true;
  }

  return res.sucess;
}

// topic callback a for listening to the pose message from the turtle
void TurtleControl::poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
  if ((msg->x >= WINDOW_EDGE) || (msg->y >= WINDOW_EDGE) || (msg->x <= 0) || (msg->y <= 0))
  {
    turtlesim::TeleportAbsolute teleport_srv;
    teleport_srv.request.theta = 0;
    teleport_srv.request.x = WINDOW_CENTER;
    teleport_srv.request.y = WINDOW_CENTER;
    teleport_client_.call(teleport_srv);

    std_srvs::Empty empty;
    clear_client.call(empty);
  }
  this->pose_msg_ = *msg;
}

// publishing the configured velocity
void TurtleControl::publish()
{
  velocity_pub_.publish(velocity_msg_);
}

// frequency setup
double TurtleControl::getRate()
{
  return PUB_RATE;
}

// getter if drawing is enabled or not
bool TurtleControl::getDrawingStatus()
{
  return this->drawing_status_;
}