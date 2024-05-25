#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

class GridOdom : public rclcpp::Node
{
 public:
  GridOdom(/* args */);
  ~GridOdom();
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr final_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr grid_odom_pub_;
  void finalPoseCallback(const nav_msgs::msg::Odometry::SharedPtr final_pose_msg);

 private:
  /* data */
};

GridOdom::GridOdom(/* args */) : Node("grid_odom_node")
{
  final_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "robot1/final_pose", 10, std::bind(&GridOdom::finalPoseCallback, this, std::placeholders::_1));
  grid_odom_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("robot1/grid_odom", 10);
}

GridOdom::~GridOdom() 
{
}

void GridOdom::finalPoseCallback(const nav_msgs::msg::Odometry::SharedPtr final_pose_msg)
{
  double scaling_width = 0.00440625;     // m/px width
  double scaling_height = 0.004520833;    // m/px height

  double world_coords_x;
  double world_coords_y;
  world_coords_x = final_pose_msg->pose.pose.position.x;
  world_coords_y = final_pose_msg->pose.pose.position.y;

  double x_scaled = world_coords_x / scaling_width;
  double y_scaled = world_coords_y / scaling_height;

  int w_grid_x;
  int w_grid_y;

  if (x_scaled - (int)x_scaled >= 0.5)
  {
    w_grid_x = (int)x_scaled + 1;
  }
  else
  {
    w_grid_x = (int)x_scaled;
  }

  if (y_scaled - (int)y_scaled >= 0.5)
  {
    w_grid_y = (int)y_scaled + 1;
  }
  else
  {
    w_grid_y = (int)y_scaled;
  }

  // 축 변화가 있을 수도 있으니까 변수 따로 받음.
  int final_grid_x = w_grid_x;
  int final_grid_y = w_grid_y;

  std_msgs::msg::Int16MultiArray grid_odom;
  grid_odom.data.push_back(final_grid_x);
  grid_odom.data.push_back(final_grid_y);

  grid_odom_pub_->publish(grid_odom);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridOdom>());
  rclcpp::shutdown();
  return 0;
}