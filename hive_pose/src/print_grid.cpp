#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

class GridTest : public rclcpp::Node
{
 public:
  GridTest(/* args */);
  ~GridTest();
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr grid_odom_sub_;

 private:
  /* data */
};

GridTest::GridTest(/* args */) : Node("print_grid_odom_node")
{
  grid_odom_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "robot1/grid_odom", 10, [this](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Grid Odom: [%d, %d]", msg->data[0], msg->data[1]);
      });
}

GridTest::~GridTest()
{
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridTest>());
  rclcpp::shutdown();
  return 0;
}