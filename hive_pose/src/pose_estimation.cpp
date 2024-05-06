#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "eigen3/Eigen/Dense"

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
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr camera_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr final_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void ekfOdomCallback(const nav_msgs::msg::Odometry::SharedPtr ekf_msg);
  void cameraOdomCallback(const std_msgs::msg::Float64MultiArray::SharedPtr camera_msg);
  void timerCallback();

 private:
  /* data */
  double yolo_confidence = 0.5;
  int robot_number_;
  Pose pose_from_ekf_;
  Pose pose_from_camera_;
  Pose final_pose_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
};

PoseEstimationNode::PoseEstimationNode(/* args */) : Node("final_pose_estimation_node")
{
  this->declare_parameter<std::int8_t>("robot_num", 1);
  robot_number_ = this->get_parameter("robot_num").as_int();
  std::string ekf_topic_name = "robot" + std::to_string(robot_number_) + "/ekf_odom";
  std::string pose_topic_name = "robot" + std::to_string(robot_number_) + "/final_pose";

  ekf_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      ekf_topic_name, 10, std::bind(&PoseEstimationNode::ekfOdomCallback, this, std::placeholders::_1));
  camera_odom_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "detection_info", 10, std::bind(&PoseEstimationNode::cameraOdomCallback, this, std::placeholders::_1));
  final_pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(pose_topic_name, 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PoseEstimationNode::timerCallback, this));
    // Declare and acquire `target_frame` parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "camera1_link");

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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

void PoseEstimationNode::cameraOdomCallback(const std_msgs::msg::Float64MultiArray::SharedPtr camera_msg)
{
  // Transform world -> camera1_link
  std::string fromFrameRel = target_frame_.c_str();
  std::string toFrameRel = "world";

  geometry_msgs::msg::TransformStamped t;
  Eigen::Isometry3d transform_matrix;

  Eigen::Vector3d camera_pose;
  if (camera_msg->data[0] == robot_number_)
  {
    camera_pose.x() = camera_msg->data[1];
    camera_pose.y() = camera_msg->data[2];
    yolo_confidence = camera_msg->data[3]; // confidence

  }
  // Look up for the transformation between target_frame and turtle2 frames
  // and send velocity commands for turtle2 to reach target_frame
  try {
    t = tf_buffer_->lookupTransform(
      toFrameRel, fromFrameRel,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      this->get_logger(), "Could not transform %s to %s: %s",
      toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    return;
  }
  transform_matrix = tf2::transformToEigen(t);
  Eigen::Vector3d world_pose = transform_matrix * camera_pose;

  pose_from_camera_.setX(world_pose.x());
  pose_from_camera_.setY(world_pose.y());
  pose_from_camera_.setYaw(0);

  RCLCPP_INFO(this->get_logger(), "camera pose: %f, %f", pose_from_camera_.getX(), pose_from_camera_.getY());
}

void PoseEstimationNode::timerCallback()
{
  // linear interpolation
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