#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

auto createQuaternionMsgFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

struct Pose
{
 public:
  double getX() const { return x_; }
  double getY() const { return y_; }
  double getYaw() const { return yaw_; }

  void setX(double x) { x_ = x; }
  void setY(double y) { y_ = y; }
  void setYaw(double yaw) { yaw_ = yaw; }

 private:
  double x_;
  double y_;
  double yaw_;
};

class PoseEstimationNode : public rclcpp::Node
{
 public:
  PoseEstimationNode(/* args */);
  ~PoseEstimationNode();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_sub_;
  // TODO: 다현이가 감싸주는 메세지 타입으로 교체. 현재 그냥 예시로 nav_msgs::msg::Odometry 사용
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr camera_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr final_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void ekfOdomCallback(const nav_msgs::msg::Odometry::SharedPtr ekf_msg);
  // TODO: 다현이가 감싸주는 메세지 타입으로 교체. 현재 그냥 예시로 nav_msgs::msg::Odometry 사용
  void cameraOdomCallback(const nav_msgs::msg::Odometry::SharedPtr camera_msg);
  void timerCallback();

 private:
  /* data */
  Pose pose_from_ekf_;
  Pose pose_from_camera_;
  Pose final_pose_;
};

PoseEstimationNode::PoseEstimationNode(/* args */) : Node("final_pose_estimation_node")
{
  ekf_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "robot1/ekf_odom", 10, std::bind(&PoseEstimationNode::ekfOdomCallback, this, std::placeholders::_1));
  // TODO: 다현이가 감싸주는 메세지 타입으로 교체. 현재 그냥 예시로 nav_msgs::msg::Odometry 사용
  camera_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "robot1/camera_odom", 10, std::bind(&PoseEstimationNode::cameraOdomCallback, this, std::placeholders::_1));
  final_pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("robot1/final_pose", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PoseEstimationNode::timerCallback, this));
}

PoseEstimationNode::~PoseEstimationNode()
{
}

void PoseEstimationNode::ekfOdomCallback(const nav_msgs::msg::Odometry::SharedPtr ekf_msg)
{
  pose_from_ekf_.setX(ekf_msg->pose.pose.position.x);
  pose_from_ekf_.setY(ekf_msg->pose.pose.position.y);
  pose_from_ekf_.setYaw(ekf_msg->pose.pose.orientation.z);
}

void PoseEstimationNode::cameraOdomCallback(const nav_msgs::msg::Odometry::SharedPtr camera_msg)
{
  // TODO: camera coords 좌표 받아서 Transform 찾고, world에서 본 좌표 계산
}

void PoseEstimationNode::timerCallback()
{
  // linear interpolation
  double yolo_confidence = 0.5;
  double alpha = yolo_confidence * 0.8;

  final_pose_.setX(alpha * pose_from_camera_.getX() + (1 - alpha) * pose_from_ekf_.getX());
  final_pose_.setY(alpha * pose_from_camera_.getY() + (1 - alpha) * pose_from_ekf_.getY());
  final_pose_.setYaw(pose_from_ekf_.getYaw());

  // final pose publish
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = this->now();
  odom_msg.header.frame_id = "robot1_odom";
  odom_msg.child_frame_id = "robot1_base_link";
  odom_msg.pose.pose.position.x = final_pose_.getX();
  odom_msg.pose.pose.position.y = final_pose_.getY();
  odom_msg.pose.pose.position.z = 0;
  odom_msg.pose.pose.orientation = createQuaternionMsgFromYaw(final_pose_.getYaw());
  final_pose_pub_->publish(odom_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseEstimationNode>());
  rclcpp::shutdown();
  return 0;
}