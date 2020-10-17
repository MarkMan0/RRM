#include "rrm_cv3_pal/forward_kinematics.h"

using namespace matrix;

ForwardKinematics::ForwardKinematics()
{
  // ROS node handler
  ros::NodeHandle n;

  // Resize joint_state array and initialize with value 0
  joint_state_.position.resize(4, 0);

  ball2_pos_.setZ(L1);
  ball3_pos_.setZ(2 * L1);
  ball4_pos_.setZ(3 * L1);
  position_.setZ(3 * L1 + L2);

  // Creating subscriber, publisher and service server
  joint_sub_ = n.subscribe("joint_states", 5, &ForwardKinematics::jointCallback, this);
}

void ForwardKinematics::broadcastDirectTransforms()
{
  tf::Transform transform;
  tf::Quaternion q;

  // base to ball_1 just rotation
  transform.setOrigin(tf::Vector3(0, 0, 0.0));
  q.setRPY(0, 0, joint_state_.position[0]);
  transform.setRotation(q);
  broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "link_ball_1"));

  // direct transforms
  // to ball_2
  transform.setOrigin(ball2_pos_);
  transform.setRotation(ball2_rot_);
  broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "link_ball_2"));

  // to ball_3
  transform.setOrigin(ball3_pos_);
  transform.setRotation(ball3_rot_);
  broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "link_ball_3"));

  // to ball_4
  transform.setOrigin(ball4_pos_);
  transform.setRotation(ball4_rot_);
  broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "link_ball_4"));

  // to tool_0
  transform.setOrigin(position_);
  transform.setRotation(orientation_);
  broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "tool0"));
}

void ForwardKinematics::broadcastVisualTransforms()
{
  tf::Transform transform;
  tf::Quaternion q;

  // Visual Links
  transform.setOrigin(tf::Vector3(0, 0, 0.1015));
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "link_ball_1", "link_vis_1"));

  transform.setOrigin(tf::Vector3(0, 0, 0.089));
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "link_ball_2", "link_vis_2"));

  transform.setOrigin(tf::Vector3(0, 0, 0.089));
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "link_ball_3", "link_vis_3"));

  transform.setOrigin(tf::Vector3(0, 0, -0.1));
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "link_ball_3", "link_vis_3B"));

  transform.setOrigin(tf::Vector3(0, 0, 0.089));
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "link_ball_4", "link_vis_4"));
}

void ForwardKinematics::broadcastTf()
{
  // Create transformation, set origin and rotation and finally send
  broadcastDirectTransforms();
  broadcastVisualTransforms();
}

static tf::Matrix3x3 rot2tfMat(const Eigen::MatrixXd& eigen_mat)
{
  tf::Matrix3x3 tf_mat;
  tf_mat.setValue(
      static_cast<double>(eigen_mat(0, 0)), static_cast<double>(eigen_mat(0, 1)), static_cast<double>(eigen_mat(0, 2)),
      static_cast<double>(eigen_mat(1, 0)), static_cast<double>(eigen_mat(1, 1)), static_cast<double>(eigen_mat(1, 2)),
      static_cast<double>(eigen_mat(2, 0)), static_cast<double>(eigen_mat(2, 1)), static_cast<double>(eigen_mat(2, 2)));

  return tf_mat;
}

static void matxd2Vector3(const Eigen::MatrixXd& mat, tf::Vector3& vec)
{
  vec.setX(mat(0, 0));
  vec.setY(mat(1, 0));
  vec.setZ(mat(2, 0));
}

void ForwardKinematics::jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint_state_ = *msg;

  Eigen::MatrixXd mat_to_joint_2 =
      createRz(joint_state_.position[0]) * createTz(L1) * createRy(joint_state_.position[1]);
  Eigen::MatrixXd mat_to_joint_3 = mat_to_joint_2 * createTz(L1) * createTz(joint_state_.position[2]);
  Eigen::MatrixXd mat_to_joint_4 = mat_to_joint_3 * createTz(L1) * createRy(joint_state_.position[3]);

  // convert rotation matrix to tf matrix
  tf::Matrix3x3 tf3d_ball2 = rot2tfMat(mat_to_joint_2);
  tf::Matrix3x3 tf3d_ball3 = rot2tfMat(mat_to_joint_3);
  tf::Matrix3x3 tf3d_ball4 = rot2tfMat(mat_to_joint_4);

  // Convert to quternion

  // joint 2 position
  Eigen::MatrixXd p(4, 1);
  p(0, 0) = 0;
  p(1, 0) = 0;
  p(2, 0) = 0;
  p(3, 0) = 1;
  Eigen::MatrixXd joint_2_pos = mat_to_joint_2 * p, joint_3_pos = mat_to_joint_3 * p, joint_4_pos = mat_to_joint_4 * p;

  matxd2Vector3(joint_2_pos, ball2_pos_);
  tf3d_ball2.getRotation(ball2_rot_);

  matxd2Vector3(joint_3_pos, ball3_pos_);
  tf3d_ball3.getRotation(ball3_rot_);

  matxd2Vector3(joint_4_pos, ball4_pos_);
  tf3d_ball4.getRotation(ball4_rot_);

  p(0, 0) = 0;
  p(1, 0) = 0;
  p(2, 0) = L2;
  p(3, 0) = 1;
  Eigen::MatrixXd result = mat_to_joint_4 * p;

  matxd2Vector3(result, position_);
  tf3d_ball4.getRotation(orientation_);
}
