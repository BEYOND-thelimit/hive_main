#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sstream>
#include <vector>
#include <cmath>
#include <iostream>

using namespace std;

// ARANGE Template (like in Python)
// start부터 stop까지 step 간격으로 숫자 배열을 생성하는 템플릿 함수
template <typename T>
vector<T> arange(T start, T stop, T step)
{
    vector<T> values;
    for (T i = start; i <= stop + 0.0001f; i += step)
    {
        values.push_back(i);
    }
    return values;
}

class TrajectoryPublisherNode : public rclcpp::Node
{
public:
    TrajectoryPublisherNode(const std::string &robot_name)
        : Node("trajectory_publisher"), robot_name_(robot_name), position_x_(0.0), position_y_(0.0), current_goal_idx_(0)
    {
        // 경로 초기화
        generateTrajectory(0.0044*8); // 0.0044f * 4.0을 사용하여 스텝 크기를 설정

        // 퍼블리셔 생성
        target_position_publisher_ = this->create_publisher<std_msgs::msg::String>("/" + robot_name_ + "/target_position", 10);

        // 오도메트리 구독자 생성
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/" + robot_name_ + "/odom", 10, std::bind(&TrajectoryPublisherNode::odomCallback, this, std::placeholders::_1));

        // 타이머를 생성하여 일정 간격으로 경로를 퍼블리시
        timer_ = this->create_wall_timer(
            100ms, std::bind(&TrajectoryPublisherNode::publishTrajectory, this));

        // 생성된 경로의 좌표 출력
        printTrajectory();
    }

private:
    // 오도메트리 콜백 함수
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        position_x_ = msg->pose.pose.position.x;
        position_y_ = msg->pose.pose.position.y;
    }

    // 경로 생성 함수
    void generateTrajectory(float step_size)
    {
        // 첫 번째 구간: (0.0,0.0) -> (0.497803,0.497803)
        float start_x = 0.0f, start_y = 0.0f;
        float mid_x = 0.497803f, mid_y = 0.497803f;
        float end_x = 1.0f, end_y = 0.0f;

        // 첫 번째 구간: (0.0,0.0) -> (0.497803,0.497803)
        float distance = hypot(mid_x - start_x, mid_y - start_y);
        for (float i = step_size; i <= distance; i += step_size) // start_x, start_y를 발행하지 않음
        {
            float t = i / distance;
            float x = start_x + t * (mid_x - start_x);
            float y = start_y + t * (mid_y - start_y);
            trajectory_.emplace_back(x, y);
        }

        // 두 번째 구간: (0.497803,0.497803) -> (1.0,0.0)
        distance = hypot(end_x - mid_x, end_y - mid_y);
        for (float i = step_size; i <= distance; i += step_size)
        {
            float t = i / distance;
            float x = mid_x + t * (end_x - mid_x);
            float y = mid_y + t * (end_y - mid_y);
            trajectory_.emplace_back(x, y);
        }

        // 마지막 점을 추가하여 정확히 end_x, end_y로 맞춥니다.
        trajectory_.emplace_back(end_x, end_y);
    }

    // 경로를 출력하는 함수
    void printTrajectory()
    {
        std::cout << "Generated trajectory: " << std::endl;
        for (const auto &point : trajectory_)
        {
            std::cout << "[" << point.first << "," << point.second << "]" << std::endl;
        }
    }

    // 경로 퍼블리시 함수
    void publishTrajectory()
    {
        if (current_goal_idx_ >= trajectory_.size())
        {
            RCLCPP_INFO(this->get_logger(), "All goals have been published.");
            return;
        }

        auto [goal_x, goal_y] = trajectory_[current_goal_idx_];
        
        // 현재 위치와 목표 위치 사이의 거리 계산
        float distance_to_goal = std::hypot(goal_x - position_x_, goal_y - position_y_);
        float goal_distance = std::hypot(goal_x - (current_goal_idx_ == 0 ? 0.0 : trajectory_[current_goal_idx_ - 1].first),
                                         goal_y - (current_goal_idx_ == 0 ? 0.0 : trajectory_[current_goal_idx_ - 1].second));

        // 목표 위치 메시지를 생성하고 퍼블리시
        std_msgs::msg::String msg;
        std::ostringstream ss;
        ss << goal_x << "," << goal_y << "," << 0 << "," << 0;
        msg.data = ss.str();
        target_position_publisher_->publish(msg);

        // 현재 목표 지점 출력
        std::cout << "Current goal: (" << goal_x << ", " << goal_y << "), Current position: (" << position_x_ << ", " << position_y_ << ")" << std::endl;

        // 에러가 5% 미만일 때만 다음 목표 지점으로 전환
        if (distance_to_goal / goal_distance < 0.5)
        {
            current_goal_idx_++;
            std::cout << "Goal reached. Moving to next goal." << std::endl;
        }
    }

    std::string robot_name_; // 로봇 이름
    std::vector<std::pair<float, float>> trajectory_; // 경로 저장
    size_t current_goal_idx_; // 현재 목표 인덱스
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_position_publisher_; // 목표 위치 퍼블리셔
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_; // 오도메트리 구독자
    rclcpp::TimerBase::SharedPtr timer_; // 타이머
    float position_x_, position_y_; // 현재 위치
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string robot_name = "robot1"; // "robot2", "robot3" 등으로 변경
    auto node = std::make_shared<TrajectoryPublisherNode>(robot_name);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
