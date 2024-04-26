#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

class GridOdom : public rclcpp::Node
{
 public:
  GridOdom(/* args */);
  ~GridOdom();
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr final_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr grid_odom_pub_;
  void finalPoseCallback(const nav_msgs::msg::Odometry::SharedPtr final_pose_msg);

 private:
  /* data */
};

GridOdom::GridOdom(/* args */) : Node("grid_odom_node")
{
  final_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "robot1/final_pose", 10, std::bind(&GridOdom::finalPoseCallback, this, std::placeholders::_1));
  grid_odom_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("robot1/grid_odom", 10);
}

GridOdom::~GridOdom()
{
}

void GridOdom::finalPoseCallback(const nav_msgs::msg::Odometry::SharedPtr final_pose_msg)
{
  //TODO: grid map에 대한 odom 계산 후 publish하는 코드 작성. 아니면 final_pose 구하는 노드에서 한 번에 계산할 생각도 해야할 듯.
  double scaling_width = 0.0074;     // Real: 1px = 0.0074m
  double scaling_height = 0.0058;    // Real: 1px = 0.0058m

  double world_coords_x = 0;
  double world_coords_y = 0;
  world_coords_x = final_pose_msg->pose.pose.position.x;
  world_coords_y = final_pose_msg->pose.pose.position.y;

  int grid_x = world_coords_x / scaling_width;
  int grid_y = world_coords_y / scaling_height;

  std_msgs::msg::UInt8MultiArray grid_odom;
  grid_odom.data.push_back(grid_x);
  grid_odom.data.push_back(grid_y);

  grid_odom_pub_->publish(grid_odom);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridOdom>());
  rclcpp::shutdown();
  return 0;
}