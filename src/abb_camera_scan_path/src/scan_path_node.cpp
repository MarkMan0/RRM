#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "abb_camera_scan_path/moveit_visual_tools.h"

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

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to move to start position");

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
  move_group.setPoseTarget(target_pose);

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    move_group.move();
  }

  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to begin scanning");

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose);
  const auto old_y = target_pose.position.y;
  target_pose.position.y = 1.4;
  waypoints.push_back(target_pose);
  target_pose.position.x = 0.76;
  waypoints.push_back(target_pose);
  target_pose.position.y = old_y;
  waypoints.push_back(target_pose);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), PLANNING_GROUP);
  rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  success = iptp.computeTimeStamps(rt, 0.05);
  rt.getRobotTrajectoryMsg(trajectory);
  move_group.execute(trajectory);
}
