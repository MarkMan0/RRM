#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "abb_camera_scan_path/moveit_visual_tools.h"

constexpr double VELOCITY_SCALE = 0.01;

void execute_waypoints(const std::vector<geometry_msgs::Pose>&, moveit::planning_interface::MoveGroupInterface&,
                       const std::string&);
void move_to_pose(const geometry_msgs::Pose&, moveit::planning_interface::MoveGroupInterface&);
void execute_path();

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_path_node");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  execute_path();
  ros::shutdown();
  return 0;
}

void execute_path()
{
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to move to start position");
  ROS_INFO("Scanning....");

  geometry_msgs::Pose target_pose;
  tf2::Quaternion q;
  q.setRPY(0, M_PI_2, M_PI_2);

  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  target_pose.orientation.w = q.w();
  target_pose.position.x = 1.10313;
  target_pose.position.y = 0.135275;
  target_pose.position.z = 1.29439;

  move_to_pose(target_pose, move_group);

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose);
  const auto old_y = target_pose.position.y;
  target_pose.position.y = 1.4;
  waypoints.push_back(target_pose);
  target_pose.position.x = 0.76;
  waypoints.push_back(target_pose);
  target_pose.position.y = old_y;
  waypoints.push_back(target_pose);

  execute_waypoints(waypoints, move_group, PLANNING_GROUP);
  waypoints.clear();

  target_pose.position.x = 0.120424;
  target_pose.position.y = 1.63386;
  target_pose.position.z = 0.726116;
  target_pose.orientation.x = -0.212347;
  target_pose.orientation.y = 0.507128;
  target_pose.orientation.z = -0.320687;
  target_pose.orientation.w = 0.771291;

  move_to_pose(target_pose, move_group);

  target_pose.position.x = 1.16754;
  target_pose.position.y = 1.4;
  target_pose.position.z = 0.6868;

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "link_6";
  ocm.header.frame_id = "base_link";
  ocm.orientation = target_pose.orientation;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);
  move_group.setMaxVelocityScalingFactor(VELOCITY_SCALE);
  move_to_pose(target_pose, move_group);
  move_group.clearPathConstraints();
  move_group.setMaxVelocityScalingFactor(1.0);
  test_constraints.orientation_constraints.clear();

  return;

  move_group.setStartState(*move_group.getCurrentState());
  target_pose.position.x = 1.49947;
  target_pose.position.y = -0.211783;
  target_pose.position.z = 0.711409;
  target_pose.orientation.x = 0.2102;
  target_pose.orientation.y = 0.5540;
  target_pose.orientation.z = -0.3805;
  target_pose.orientation.w = 0.709949;
  move_to_pose(target_pose, move_group);

  target_pose.position.x = 0.4498;
  target_pose.position.y = -0.3801;
  target_pose.position.z = 0.6925;
  ocm.orientation = target_pose.orientation;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);
  move_group.setMaxVelocityScalingFactor(VELOCITY_SCALE);
  move_to_pose(target_pose, move_group);
  move_group.clearPathConstraints();
  move_group.setMaxVelocityScalingFactor(1.0);
  test_constraints.orientation_constraints.clear();
}

void move_to_pose(const geometry_msgs::Pose& pose, moveit::planning_interface::MoveGroupInterface& move_group)
{
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group.setPoseTarget(pose);

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    move_group.move();
  }
}

void execute_waypoints(const std::vector<geometry_msgs::Pose>& waypoints,
                       moveit::planning_interface::MoveGroupInterface& move_group, const std::string& group)
{
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), group);
  rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  bool success = iptp.computeTimeStamps(rt, VELOCITY_SCALE);
  rt.getRobotTrajectoryMsg(trajectory);
  move_group.execute(trajectory);
}
