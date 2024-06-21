#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>


#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <algorithm>

#include "std_msgs/msg/string.hpp"
#include <sstream>
#include <vector>

// 해야할 것: 골 바라보고 출발, 골 도착 안했으면 goal_state false, 도착하면 true
using namespace std;

#define DEG2RAD(a) ((a) * (M_PI / 180.0f))

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

class DWANode : public rclcpp::Node
{
public:
    DWANode()
        : Node("dwa_planner"), target_x(0.0f), target_y(0.0f), way_point(0), goal_arrived_flag(false), turn_flag(false), // last_way_point(0),
          position_x(0.0f), position_y(0.0f), robot_theta(0.0f), last_target_x(-123.0f), last_target_y(-123.0f),         // last_target임의로 처음과 다른 지점 주는 거임
          lidar_x(nullptr), lidar_y(nullptr), ranges(nullptr), angle_difference(0.0f),
          v_ref(0.0f), omega_ref(0.0f),
          v_current(0.0f), omega_current(0.0f),
          firstinit(false)
    {
        // 목표 위치 및 파라미터 선언 및 초기화
        this->declare_parameter<float>("target_x", 0.0);
        this->declare_parameter<float>("target_y", 0.0);
        this->get_parameter("target_x", target_x);
        this->get_parameter("target_y", target_y);

        // target_position 구독
        target_position_subscription = this->create_subscription<std_msgs::msg::String>(
            "/robot1/target_position", 10,
            std::bind(&DWANode::targetPositionCallback, this, std::placeholders::_1));

        // goal_status 발행
        goal_status_publisher = this->create_publisher<std_msgs::msg::String>("/robot1/goal_status", 10);

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot1/odom", rclcpp::SensorDataQoS(), std::bind(&DWANode::odom_callback, this, std::placeholders::_1));

        // "/robot1/scan" 토픽에 대한 구독자를 생성합니다. QoS 설정으로 rclcpp::SensorDataQoS()를 사용합니다.
        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/robot1/converted_scan", rclcpp::SensorDataQoS(), std::bind(&DWANode::laser_callback, this, std::placeholders::_1));

        // "/robot1/cmd_vel" 토픽에 Twist 메시지를 발행하는 발행자를 생성합니다. 발행 큐의 크기는 10으로 설정됩니다.
        cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            100ms, [this]()
            { this->compute(); });
    }

    // Destructor
    ~DWANode()
    {
        delete[] ranges;
        delete[] lidar_x;
        delete[] lidar_y;
    }

    // 로봇의 위치와 방향 정보를 업데이트하는 콜백 함수
    void odom_callback(const nav_msgs::msg::Odometry::ConstPtr &msg)
    {

        position_x = msg->pose.pose.position.x;
        position_y = msg->pose.pose.position.y;
        robot_theta = 2.0f * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        linear_velocity_x = msg->twist.twist.linear.x;  // 선속도 x
        angular_velocity_z = msg->twist.twist.angular.z;  // 각속도 z
        degree_repair(robot_theta); // 각도를 [-π, π] 범위로 정규화
    }

    // 라이다 데이터를 처리하는 콜백 함수
    void laser_callback(const sensor_msgs::msg::LaserScan::ConstPtr &msg)
    {

        lidar_flag = false;
        lidar_N = (int)((msg->angle_max - msg->angle_min) / msg->angle_increment) + 1;

        // first initialization - locate memory for table
        if (firstinit == false)
        {
            ranges = new float[lidar_N];
            lidar_x = new float[lidar_N];
            lidar_y = new float[lidar_N];
            firstinit = true;
        }
        // Calculate obstacle global position X/Y and min range from the obstacle
        float min_value = 9999;
        for (int i = 0; i < lidar_N; i++)
        {
            ranges[i] = msg->ranges[i];
            if (!isnan(ranges[i]) && !isinf(ranges[i]) && ranges[i] > min_range)
            {
                if (ranges[i] < min_value)
                {
                    // 제일 가까운 index 저장
                    min_index = i;
                    min_value = ranges[i];
                }

                float xl = cos(msg->angle_min + i * msg->angle_increment) * ranges[i];
                float yl = sin(msg->angle_min + i * msg->angle_increment) * ranges[i];

                // obstacle global position
                lidar_x[i] = position_x + xl * cos(robot_theta) - yl * sin(robot_theta);
                lidar_y[i] = position_y + xl * sin(robot_theta) + yl * cos(robot_theta);
            }
        }
        lidar_flag = (min_value != 9999);
    }

    // 로봇의 각도를 [-π, π] 범위로 정규화하는 함수
    void degree_repair(float &robot_theta)
    {

        if (robot_theta > DEG2RAD(180))
        {
            robot_theta -= DEG2RAD(360);
        }
        else if (robot_theta < (-1.0f) * DEG2RAD(180))
        {
            robot_theta += DEG2RAD(360);
        }
    }

    std::vector<std::string> split(const std::string &s, char delimiter)
    {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delimiter))
        {
            tokens.push_back(token);
        }
        return tokens;
    }

    void targetPositionCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::vector<std::string> parsed_data = split(msg->data, ',');
        if (parsed_data.size() < 4)
            return; // 데이터 검증

        target_x = std::stod(parsed_data[0]);
        target_y = std::stod(parsed_data[1]);
        way_point = std::stoi(parsed_data[2]);
        last_arrived = std::stoi(parsed_data[3]) != 0; // 도착 여부, 0이 아니면 true

        if (target_x != last_target_x || target_y != last_target_y)
        {
            // last_way_point = way_point;
            target_flag = true;
            goal_arrived_flag = false;
            turn_flag = true;
            last_target_x = target_x;
            last_target_y = target_y;
            v_current = 0.0;
            omega_current = 0.0;
        }
    }
    // 로봇이 목표 위치를 바라볼 때까지 회전하는 함수
    void rotateTowardsTarget(double robot_theta, double position_x, double position_y, double target_x, double target_y)
    {
        // 목표 방향 계산
        double target_direction = atan2(target_y - position_y, target_x - position_x);

        // 현재 방향과 목표 방향 사이의 각도 차이 계산
        angle_difference = target_direction - robot_theta;

        // 각도 차이를 -π ~ π 범위로 정규화
        while (angle_difference > M_PI)
            angle_difference -= 2 * M_PI;
        while (angle_difference < -M_PI)
            angle_difference += 2 * M_PI;
        // 회전해야 하는 방향과 각도에 따라 로봇을 회전시키는 명령 발행

        // 처음에 회전하고 출발하는 부분 여기 넣음
        geometry_msgs::msg::Twist cmd_msg;
        if (std::abs(angle_difference) > 0.035)
        { // 회전해야 할 각도가 임계값(예: 0.01 라디안)보다 큰 경우
            // 선속도를 0으로 설정하여 현재 위치에서 회전만 수행
            cmd_msg.linear.x = 0.0;
            // 각속도 설정, 각도 차이에 따라 각속도를 결정 (여기에서는 단순화된 예시)
            cmd_msg.angular.z = angle_difference > 0 ? 0.5 : -0.5; // 각속도를 적절히 조절하세요
        }
        else
        {
            turn_flag = false;
            RCLCPP_INFO(this->get_logger(), "Turn flag == false.");
            // 목표 방향을 향하고 있을 때의 처리, 필요하다면 선속도를 조절하여 전진하도록 설정
            cmd_msg.linear.x = 0.0; // 필요에 따라 변경
            cmd_msg.angular.z = 0.0;
        }
        cmd_pub->publish(cmd_msg);
    }

    void goalObstacleCheckFunc()
    {
        // goal 위치에 장애물이 있다면 대기. 로봇이 goal에 대해 10도 이내로 바라보고, 거리가 0.9미터 이내라면 장애물이 goal에 있다고 판별
        if (lidar_flag == true && abs(robot_theta - atan2(lidar_y[min_index] - position_y, lidar_x[min_index] - position_x)) < DEG2RAD(10) && abs(robot_theta - atan2(target_y - position_y, target_x - position_x)) < DEG2RAD(10) && hypot(lidar_x[min_index] - position_x, lidar_y[min_index] - position_y) < 0.9 && hypot(lidar_x[min_index] - position_x, lidar_y[min_index] - position_y) < 0.9 && hypot(target_x - position_x, target_y - position_y) < 1.5)
        {
            goal_obstacle_check = true;
        }
        else
        {
            goal_obstacle_check = false;
        }
        if (goal_obstacle_check == true)
        {
            geometry_msgs::msg::Twist cmd_msg;
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = 0.0;
            cmd_pub->publish(cmd_msg);
        }
    }

    // Calculating DWA
    void compute()
    {
        // std::cout << "target_flag: " << to_string(target_flag) << " / "
        //           << "goal_arrived_flag: " << to_string(goal_arrived_flag) << " / "
        //           << "turn_flag: " << to_string(turn_flag) << std::endl;
        // std::cout << "way_point: " << way_point << " / "
        //           << "last_way_point: " << last_way_point << std::endl;
        goalObstacleCheckFunc();
        // 방향 맞추고 출발해야 한다면 돌기
        if (target_flag == true && turn_flag == true)
        {
            // 목표 방향 계산
            float target_direction = atan2(target_y - position_y, target_x - position_x);
            // 현재 방향과 목표 방향 사이의 각도 차이 계산
            float angle_difference = target_direction - robot_theta;

            // 각도 차이를 -π ~ π 범위로 정규화
            while (angle_difference > M_PI)
                angle_difference -= 2 * M_PI;
            while (angle_difference < -M_PI)
                angle_difference += 2 * M_PI;

            // 40도보다 큰 각도 차이가 있는 경우에만 방향 조정
            if (std::abs(angle_difference) > DEG2RAD(40)) // 40도를 라디안으로 변환
            {
                rotateTowardsTarget(robot_theta, position_x, position_y, target_x, target_y);
                // RCLCPP_INFO(this->get_logger(), "turning");
            }
            else
            {
                turn_flag = false; // 필요한 경우 플래그 업데이트
                // RCLCPP_INFO(this->get_logger(), "minor angle difference, not turning");
            }
        }
        else if (goal_arrived_flag == false && target_flag == true && turn_flag == false && goal_obstacle_check == false)
        {
            if (v_current < 0) //후진을 하고 다시 경로 탐색하는 경우 이렇게 해야함
            {
                v_current = 0.0;
                omega_current = 0.0;
            }
            // RCLCPP_INFO(this->get_logger(), "DWA.");
            if (hypot(target_x - position_x, target_y - position_y) <= position_accuracy)
            {

                goal_arrived_flag = true;
            }
            else
            {
                goal_arrived_flag = false;
            }

            // calculate DynamicWindow
            float DynamicWindow[4] = {max(0.05f, v_current - a_max * dT),
                                      min(v_max, v_current + a_max * dT),
                                      max(-omega_max, omega_current - epsilon_max * dT),
                                      min(omega_max, omega_current + epsilon_max * dT)};

            // 속도와 각속도의 가능한 범위 계산
            auto v_range = arange<float>(DynamicWindow[0], DynamicWindow[1], v_accuracy_prediction);
            auto omega_range = arange<float>(DynamicWindow[2], DynamicWindow[3], omega_accuracy_prediction);
            auto t_prediction_range = arange<float>(0.0f, prediction_time, dT);
            int v_range_size = v_range.size();
            int omega_range_size = omega_range.size();
            int t_prediction_size = t_prediction_range.size();

            // 경로 시뮬레이션을 위한 4차원 벡터 준비
            vector<vector<vector<vector<float>>>> eval_trajectory(v_range_size, vector<vector<vector<float>>>(omega_range_size, vector<vector<float>>(t_prediction_size, vector<float>(3, 0.0f))));

            // 각 경로에 대한 평가 함수 점수를 저장할 벡터
            vector<vector<float>> eval_function(v_range_size, vector<float>(omega_range_size, 0));

            // 최적 경로를 찾기 위한 변수 초기화
            float opt_max_eval_fun = -999.0f;
            float opt_idx[2] = {0, 0};
            // int checkcount = 0;

            // 확인용임
            float velocity_score = 0.0;
            float heading_score = 0.0;
            float distance_score = 0.0;
            float goal_score = 0.0;

            // 미래의 경로를 시뮬레이션
            for (int i = 0; i < v_range_size; i++)
            {

                for (int j = 0; j < omega_range_size; j++)
                {

                    // 이전 상태를 기반으로 새로운 상태 계산
                    float eval_v = v_range[i];
                    float eval_omega = omega_range[j];
                    eval_trajectory[i][j][0][0] = position_x;
                    eval_trajectory[i][j][0][1] = position_y;
                    eval_trajectory[i][j][0][2] = robot_theta;
                    for (int k = 1; k < t_prediction_size; k++)
                    {

                        float x_prev = eval_trajectory[i][j][k - 1][0];
                        float y_prev = eval_trajectory[i][j][k - 1][1];
                        float theta_prev = eval_trajectory[i][j][k - 1][2];

                        eval_trajectory[i][j][k][0] = x_prev + dT * cos(theta_prev) * eval_v;
                        eval_trajectory[i][j][k][1] = y_prev + dT * sin(theta_prev) * eval_v;
                        eval_trajectory[i][j][k][2] = theta_prev + dT * eval_omega;
                    }

                    // 경로의 마지막 지점에서의 방향과 목표 지점과의 방향 차이 계산
                    float robot_theta = eval_trajectory[i][j][t_prediction_size - 1][2];
                    float goaltheta = atan2(target_y - eval_trajectory[i][j][t_prediction_size - 1][1],
                                            target_x - eval_trajectory[i][j][t_prediction_size - 1][0]);
                    degree_repair(robot_theta); // 방향 정규화

                    // 정규화한 reward [0,1]
                    float velocityReward = velocity_gain * (eval_v / v_max);
                    float headingReward = heading_gain * ((cos(abs(goaltheta - robot_theta)) + 1.0f) / 2.0f);

                    // goal이 10미터 밖에 있으면 10미터, 그 안에 있으면 거리값. 현재 목적지는 d* lite를 이용하기 때문에 10미터 이내에는 무조건 있다고 가정하에 진행. 10미터 밖에 있으면 목적지로 가지 않음.
                    float goal_nowposition_distance = (hypot(target_x - eval_trajectory[i][j][t_prediction_size - 1][0], target_y - eval_trajectory[i][j][t_prediction_size - 1][1]) < max_distance) ? hypot(target_x - eval_trajectory[i][j][t_prediction_size - 1][0], target_y - eval_trajectory[i][j][t_prediction_size - 1][1]) : max_distance;
                    float goalReward = goal_gain * (1 - (goal_nowposition_distance / max_distance));

                    float obstacle_nowposition_distance = 0.0;
                    // 장애물이 관측 범위 안에 있으면 관측 거리, 밖에 있으면 관측 허용 거리 사용
                    if (lidar_flag == false)
                    {
                        obstacle_nowposition_distance = observe_obstacle_distance;
                        // std::cout << "no obstacle" << std::endl;
                    }
                    else
                    {
                        float predict_obstacle_distance = hypot(lidar_x[min_index] - eval_trajectory[i][j][t_prediction_size - 1][0], lidar_y[min_index] - eval_trajectory[i][j][t_prediction_size - 1][1]);
                        if (predict_obstacle_distance < obstacledistance)
                        {
                            obstacle_nowposition_distance = 0.0;
                        }
                        else
                        {
                            obstacle_nowposition_distance = (predict_obstacle_distance < observe_obstacle_distance) ? hypot(lidar_x[min_index] - eval_trajectory[i][j][t_prediction_size - 1][0], lidar_y[min_index] - eval_trajectory[i][j][t_prediction_size - 1][1]) : observe_obstacle_distance;
                        }
                        // std::cout << "yes obstacle" << std::endl;
                    }
                    float distanceReward = obstacle_gain * (obstacle_nowposition_distance / observe_obstacle_distance);

                    eval_function[i][j] = headingReward + distanceReward + goalReward + velocityReward;
                    bool check = false;
                    // Check if there are obstacles on the way, ignore first 2 predictions
                    for (int k = 2; k < t_prediction_size; k = k + 1)
                    {

                        if (lidar_flag && hypot(lidar_x[min_index] - eval_trajectory[i][j][k][0], lidar_y[min_index] - eval_trajectory[i][j][k][1]) <= obstacledistance)
                        {

                            check = true;
                            // checkcount++;
                        }
                    }

                    if (eval_function[i][j] > opt_max_eval_fun && check == false)
                    {
                        opt_idx[0] = i;
                        opt_idx[1] = j;
                        opt_max_eval_fun = eval_function[i][j];
                        // 확인용
                        velocity_score = velocityReward;
                        heading_score = headingReward;
                        distance_score = distanceReward;
                        goal_score = goalReward;
                    }
                }
            }

            try
            {
                v_ref = v_range[opt_idx[0]];
                omega_ref = omega_range[opt_idx[1]];
            }
            catch (...)
            {
                v_ref = 0;
                omega_ref = 0;
            }

            if (opt_max_eval_fun <= 0) // 너무 장애물이 가까워서 최적 경로가 없는 경우
            {
                too_close_flag = true;
            }
            if (too_close_flag == true)
            {
                if (hypot(lidar_y[min_index] - position_y, lidar_x[min_index] - position_x) > (obstacledistance))
                {
                    too_close_flag = false;
                }

                if (abs(robot_theta - atan2(lidar_y[min_index] - position_y, lidar_x[min_index] - position_x)) > DEG2RAD(90))
                {
                    v_ref = 0.1;
                    omega_ref = (-1.0) * std::copysign(1.0, atan2(lidar_y[min_index] - position_y, lidar_x[min_index] - position_x) - robot_theta) * 0.2;
                    std::cout << "too close obstacle is behind robot!!" << std::endl;
                }
                else
                {
                    v_ref = -0.2;
                    omega_ref = (-1.0) * std::copysign(1.0, atan2(lidar_y[min_index] - position_y, lidar_x[min_index] - position_x) - robot_theta) * 0.1;
                    std::cout << "too close obstacle is in front of robot!!" << std::endl;
                }
            }
            else{
                std::cout << "move!!" << std::endl;
            }

            v_current = v_ref;
            omega_current = omega_ref;

            // cmd_vel 메시지를 발행하는 부분 추가
            geometry_msgs::msg::Twist cmd_msg;
            cmd_msg.linear.x = v_ref;
            cmd_msg.angular.z = omega_ref;
            cmd_pub->publish(cmd_msg);

            // 평가점수 확인용
            // std::cout << std::fixed << std::setprecision(2);
            // std::cout << "[velocity_score: " << velocity_score << "] , [heading_score: " << heading_score << "], [distance_score: " << distance_score << "] , [goal_score: " << goal_score << "]" << std::endl;
        }
        // 도착함을 publish
        else if (goal_arrived_flag == true)
        {
            // if (way_point == last_way_point)
            // {
            //     way_point += 1;
            // }
            std_msgs::msg::String goal_status_msg;
            goal_status_msg.data = std::to_string(goal_arrived_flag) + "," + std::to_string(way_point);
            goal_status_publisher->publish(goal_status_msg);

            if (last_arrived == true)
            {
                geometry_msgs::msg::Twist stop_msg;
                stop_msg.linear.x = 0.0;
                stop_msg.angular.z = 0.0;
                cmd_pub->publish(stop_msg);
            }
            // 목표에 도달했으므로 로봇을 멈춤

            RCLCPP_INFO(this->get_logger(), "Goal reached, stopping.");
            // std::cout << "way_point: " << way_point << std::endl;

            target_flag = false;
        }
    }

private:
    // Variables
    float target_x;
    float target_y;
    float last_target_x;
    float last_target_y;
    bool target_flag = false; // 이전 target과 같은지 보는것.즉, 이전 target과 다르면 움직여야 함 -> true
    bool turn_flag = true;    // 돌고 출발할거 확인용, 돌아야 하면 true
    bool goal_obstacle_check = false; // 최종 목적지에 장애물 있는지 체크
    bool goal_arrived_flag = false;


    double angle_difference;

    int way_point;
    bool last_arrived = false;        // 최종 골에 도달 했는지
    // int last_way_point;
    float position_x = 0;
    float position_y = 0;
    float linear_velocity_x = 0.0;
    float angular_velocity_z = 0.0;
    float position_accuracy = 0.0044*4*0.1;
    float obstacledistance = 0.7f;

    float max_distance = 2.0f;              // 로봇 goal까지의 최대 거리
    float observe_obstacle_distance = 2.0f; // 관측할 장애물 거리, 즉 허용 범위

    float robot_theta = 0; // 로봇 yaw
    float *lidar_x;
    float *lidar_y;
    float *ranges;

    float v_ref = 0;
    float omega_ref = 0;
    float v_current = 0;
    float omega_current = 0;

    bool lidar_flag = false;
    bool too_close_flag = false;

    // DWA Parameters
    float a_max = 0.01;
    float v_max = 0.25;
    float omega_max = 0.5;
    float epsilon_max = 0.1;

    // STEP
    float dT = 0.5;
    float v_accuracy_prediction = 0.005;
    float omega_accuracy_prediction = 0.001 * 3.14;

    // Prediction time
    float prediction_time = 2.0f;

    int lidar_N = 0;
    int min_index = 0;
    bool firstinit = false;

    // minimum laser range!!!!!
    float min_range = 0.1;

    // DWA gain
    float goal_gain = 1.0f;
    float velocity_gain = 1.0f;
    float heading_gain = 1.0f;
    float obstacle_gain = 1.0f;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_position_subscription;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_status_publisher;

    // Add other function and variable declarations as necessary
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DWANode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

//******************************************************************************************************************************
// #include <rclcpp/rclcpp.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <geometry_msgs/msg/twist.hpp>

// #include <iostream>
// #include <vector>
// #include <cmath>
// #include <chrono>
// #include <algorithm>

// using namespace std;

// #define DEG2RAD(a) ((a) * (M_PI / 180.0f))

// // ARANGE Template (like in Python)
// // start부터 stop까지 step 간격으로 숫자 배열을 생성하는 템플릿 함수
// template <typename T>
// vector<T> arange(T start, T stop, T step)
// {
//     vector<T> values;
//     for (T i = start; i <= stop + 0.0001f; i += step)
//     {
//         values.push_back(i);
//     }
//     return values;
// }

// class DWANode : public rclcpp::Node
// {
// public:
//     DWANode()
//         : Node("dwa_planner"), goal_x(0.0f), goal_y(0.0f),
//           position_x(0.0f), position_y(0.0f), robot_theta(0.0f),
//           lidar_x(nullptr), lidar_y(nullptr), ranges(nullptr),
//           v_ref(0.0f), omega_ref(0.0f),
//           v_current(0.0f), omega_current(0.0f),
//           firstinit(false)
//     {
//         // 목표 위치 및 파라미터 선언 및 초기화
//         this->declare_parameter<float>("goal_x", 0.0);
//         this->declare_parameter<float>("goal_y", 0.0);
//         this->get_parameter("goal_x", goal_x);
//         this->get_parameter("goal_y", goal_y);

//         goal_sub = this->create_subscription<geometry_msgs::msg::Point>(
//             "/goal_point", rclcpp::SensorDataQoS(), std::bind(&DWANode::goal_callback, this, std::placeholders::_1));

//         odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
//             "/robot1/odom", rclcpp::SensorDataQoS(), std::bind(&DWANode::odom_callback, this, std::placeholders::_1));

//         // "/robot1/scan" 토픽에 대한 구독자를 생성합니다. QoS 설정으로 rclcpp::SensorDataQoS()를 사용합니다.
//         laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
//             "/robot1/converted_scan", rclcpp::SensorDataQoS(), std::bind(&DWANode::laser_callback, this, std::placeholders::_1));

//         // "/robot1/cmd_vel" 토픽에 Twist 메시지를 발행하는 발행자를 생성합니다. 발행 큐의 크기는 10으로 설정됩니다.
//         cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);

//         timer_ = this->create_wall_timer(
//             100ms, [this]()
//             { this->compute(); });
//     }

//     // Destructor
//     ~DWANode()
//     {
//         delete[] ranges;
//         delete[] lidar_x;
//         delete[] lidar_y;
//     }

//     // 목표 위치 변경 시 호출되는 콜백 함수
//     void goal_callback(const geometry_msgs::msg::Point::SharedPtr msg)
//     {
//         goal_x = msg->x;
//         goal_y = msg->y;
//         RCLCPP_INFO(this->get_logger(), "New goal: X=%f, Y=%f", goal_x, goal_y);
//     }

//     // 로봇의 위치와 방향 정보를 업데이트하는 콜백 함수
//     void odom_callback(const nav_msgs::msg::Odometry::ConstPtr &msg)
//     {
//         position_x = msg->pose.pose.position.x;
//         position_y = msg->pose.pose.position.y;
//         robot_theta = 2.0f * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
//         degree_repair(robot_theta); // 각도를 [-π, π] 범위로 정규화
//     }

//     // 라이다 데이터를 처리하는 콜백 함수
//     void laser_callback(const sensor_msgs::msg::LaserScan::ConstPtr &msg)
//     {
//         bool check_flag = false;

//         lidar_flag = false;
//         lidar_N = (int)((msg->angle_max - msg->angle_min) / msg->angle_increment) + 1;

//         int count = 1;

//         // first initialization - locate memory for table
//         if (firstinit == false)
//         {
//             ranges = new float[lidar_N];
//             lidar_x = new float[lidar_N];
//             lidar_y = new float[lidar_N];
//             firstinit = true;
//         }
//         // Calculate obstacle global position X/Y and min range from the obstacle

//         float min_value = 9999;
//         for (int i = 0; i < lidar_N; i++)
//         {
//             ranges[i] = msg->ranges[i];
//             if (!isnan(ranges[i]) && !isinf(ranges[i]) && ranges[i] > min_range)
//             {
//                 if (ranges[i] < min_value)
//                 {
//                     // 제일 가까운 index 저장
//                     min_index = i;
//                     min_value = ranges[i];
//                 }

//                 float xl = cos(msg->angle_min + i * msg->angle_increment) * ranges[i];
//                 float yl = sin(msg->angle_min + i * msg->angle_increment) * ranges[i];

//                 // obstacle global position
//                 lidar_x[i] = position_x + xl * cos(robot_theta) - yl * sin(robot_theta);
//                 lidar_y[i] = position_y + xl * sin(robot_theta) + yl * cos(robot_theta);
//             }
//             // 확인용
//             // if (count <= 10)
//             // {
//             //     // std::cout << " [" << lidar_x[i] << " / " << lidar_y[i] << "], ";
//             //     std::cout << " [" << ranges[i]  << "], ";
//             //     count += 1;
//             // }
//             // else
//             // {
//             //     count = 1;
//             //     std::cout << std::endl;
//             // }

//             // 확인용
//             // if (check_flag == false && ranges[i] != std::numeric_limits<float>::infinity())
//             // {
//             //     check_flag = true;
//             // }
//         }
//         // std::cout << "check_flag: " << check_flag << std::endl;
//         // std::cout << "=================================" << std::endl;
//         lidar_flag = (min_value != 9999);
//     }

//     // 로봇의 각도를 [-π, π] 범위로 정규화하는 함수
//     void degree_repair(float &robot_theta)
//     {

//         if (robot_theta > DEG2RAD(180))
//         {
//             robot_theta -= DEG2RAD(360);
//         }
//         else if (robot_theta < (-1.0f) * DEG2RAD(180))
//         {
//             robot_theta += DEG2RAD(360);
//         }
//     }

//     // Calculating DWA
//     void compute()
//     {
//         if (v_current < 0)
//         {
//             v_current = 0.0;
//         }
//         // goal 위치에 장애물이 있다면 대기. 로봇이 goal에 대해 10도 이내로 바라보고, 거리가 0.9미터 이내라면 장애물이 goal에 있다고 판별
//         if (lidar_flag == true && abs(robot_theta - atan2(lidar_y[min_index] - position_y, lidar_x[min_index] - position_x)) < DEG2RAD(10) && abs(robot_theta - atan2(goal_y - position_y, goal_x - position_x)) < DEG2RAD(10) && hypot(lidar_x[min_index] - position_x, lidar_y[min_index] - position_y) < 0.9 && hypot(goal_x - position_x, goal_y - position_y) < 1.5)
//         {
//             goal_obstacle_check = true;
//             std::cout << "obstacle is in goal boundery!!" << std::endl;
//         }
//         else
//         {
//             goal_obstacle_check = false;
//         }
//         if (goal_obstacle_check == true)
//         {
//             geometry_msgs::msg::Twist cmd_msg;
//             cmd_msg.linear.x = 0.0;
//             cmd_msg.angular.z = 0.0;
//             cmd_pub->publish(cmd_msg);
//         }

//         // 목표 지점에 도달했는지 체크
//         if (hypot(goal_x - position_x, goal_y - position_y) <= position_accuracy)
//         {
//             // 목표에 도달했으므로 로봇을 멈춤
//             geometry_msgs::msg::Twist stop_msg;
//             stop_msg.linear.x = 0.0;
//             stop_msg.angular.z = 0.0;
//             cmd_pub->publish(stop_msg);
//             // RCLCPP_INFO(this->get_logger(), "Goal reached, stopping.");
//         }

//         else if (goal_obstacle_check == false)
//         {
//             // calculate DynamicWindow
//             float DynamicWindow[4] = {max(v_min, v_current - a_max * dT),
//                                       min(v_max, v_current + a_max * dT),
//                                       max(-omega_max, omega_current - epsilon_max * dT),
//                                       min(omega_max, omega_current + epsilon_max * dT)};

//             // 속도와 각속도의 가능한 범위 계산
//             auto v_range = arange<float>(DynamicWindow[0], DynamicWindow[1], v_accuracy_prediction);
//             auto omega_range = arange<float>(DynamicWindow[2], DynamicWindow[3], omega_accuracy_prediction);
//             auto t_prediction_range = arange<float>(0.0f, prediction_time, dT);
//             int v_range_size = v_range.size();
//             int omega_range_size = omega_range.size();
//             int t_prediction_size = t_prediction_range.size();

//             // 경로 시뮬레이션을 위한 4차원 벡터 준비
//             vector<vector<vector<vector<float>>>> eval_trajectory(v_range_size, vector<vector<vector<float>>>(omega_range_size, vector<vector<float>>(t_prediction_size, vector<float>(3, 0.0f))));

//             // 각 경로에 대한 평가 함수 점수를 저장할 벡터
//             vector<vector<float>> eval_function(v_range_size, vector<float>(omega_range_size, 0));

//             // 최적 경로를 찾기 위한 변수 초기화
//             float opt_max_eval_fun = -999.0f;
//             float opt_idx[2] = {0, 0};
//             // int checkcount = 0;

//             float velocity_score = 0.0;
//             float heading_score = 0.0;
//             float distance_score = 0.0;
//             float goal_score = 0.0;

//             // 미래의 경로를 시뮬레이션
//             for (int i = 0; i < v_range_size; i++)
//             {
//                 for (int j = 0; j < omega_range_size; j++)
//                 {
//                     // 이전 상태를 기반으로 새로운 상태 계산
//                     float eval_v = v_range[i];
//                     float eval_omega = omega_range[j];
//                     eval_trajectory[i][j][0][0] = position_x;
//                     eval_trajectory[i][j][0][1] = position_y;
//                     eval_trajectory[i][j][0][2] = robot_theta;
//                     for (int k = 1; k < t_prediction_size; k++)
//                     {
//                         float x_prev = eval_trajectory[i][j][k - 1][0];
//                         float y_prev = eval_trajectory[i][j][k - 1][1];
//                         float theta_prev = eval_trajectory[i][j][k - 1][2];

//                         eval_trajectory[i][j][k][0] = x_prev + dT * cos(theta_prev) * eval_v;
//                         eval_trajectory[i][j][k][1] = y_prev + dT * sin(theta_prev) * eval_v;
//                         eval_trajectory[i][j][k][2] = theta_prev + dT * eval_omega;
//                     }

//                     // 경로의 마지막 지점에서의 방향과 목표 지점과의 방향 차이 계산
//                     float robot_theta = eval_trajectory[i][j][t_prediction_size - 1][2];
//                     float goaltheta = atan2(goal_y - eval_trajectory[i][j][t_prediction_size - 1][1],
//                                             goal_x - eval_trajectory[i][j][t_prediction_size - 1][0]);
//                     degree_repair(robot_theta); // 방향 정규화

//                     // 정규화한 reward [0,1]
//                     float velocityReward = velocity_gain * (eval_v / v_max);
//                     float headingReward = heading_gain * ((cos(abs(goaltheta - robot_theta)) + 1.0f) / 2.0f);

//                     // goal이 10미터 밖에 있으면 10미터, 그 안에 있으면 거리값. 현재 목적지는 d* lite를 이용하기 때문에 10미터 이내에는 무조건 있다고 가정하에 진행. 10미터 밖에 있으면 목적지로 가지 않음.
//                     float goal_nowposition_distance = (hypot(goal_x - eval_trajectory[i][j][t_prediction_size - 1][0], goal_y - eval_trajectory[i][j][t_prediction_size - 1][1]) < max_distance) ? hypot(goal_x - eval_trajectory[i][j][t_prediction_size - 1][0], goal_y - eval_trajectory[i][j][t_prediction_size - 1][1]) : max_distance;
//                     float goalReward = goal_gain * (1 - (goal_nowposition_distance / max_distance));

//                     float obstacle_nowposition_distance = 0.0;
//                     // 장애물이 관측 범위 안에 있으면 관측 거리, 밖에 있으면 관측 허용 거리 사용
//                     if (lidar_flag == false)
//                     {
//                         obstacle_nowposition_distance = observe_obstacle_distance;
//                         // std::cout << "no obstacle" << std::endl;
//                     }
//                     else
//                     {
//                         obstacle_nowposition_distance = (hypot(lidar_x[min_index] - eval_trajectory[i][j][t_prediction_size - 1][0], lidar_y[min_index] - eval_trajectory[i][j][t_prediction_size - 1][1]) < observe_obstacle_distance) ? hypot(lidar_x[min_index] - eval_trajectory[i][j][t_prediction_size - 1][0], lidar_y[min_index] - eval_trajectory[i][j][t_prediction_size - 1][1]) : observe_obstacle_distance;
//                         // std::cout << "yes obstacle" << std::endl;
//                     }

//                     // std::cout << "obstacle_nowposition_distance: " << obstacle_nowposition_distance << std::endl;
//                     float distanceReward = obstacle_gain * (obstacle_nowposition_distance / observe_obstacle_distance);

//                     eval_function[i][j] = headingReward + distanceReward + goalReward + velocityReward;
//                     bool check = false;
//                     // Check if there are obstacles on the way, ignore first 2 predictions
//                     for (int k = 2; k < t_prediction_size; k = k + 1)
//                     {

//                         if (lidar_flag && hypot(lidar_x[min_index] - eval_trajectory[i][j][k][0], lidar_y[min_index] - eval_trajectory[i][j][k][1]) <= obstacledistance)
//                         {
//                             check = true;
//                         }
//                     }
//                     // std::cout << "opt_max_eval_fun: " << opt_max_eval_fun << " / check: " << check << endl;

//                     if (eval_function[i][j] > opt_max_eval_fun && check == false)
//                     {
//                         opt_idx[0] = i;
//                         opt_idx[1] = j;
//                         opt_max_eval_fun = eval_function[i][j];
//                         velocity_score = velocityReward;
//                         heading_score = headingReward;
//                         distance_score = distanceReward;
//                         goal_score = goalReward;
//                     }
//                 }
//             }

//             try
//             {
//                 v_ref = v_range[opt_idx[0]];
//                 omega_ref = omega_range[opt_idx[1]];
//             }
//             catch (...)
//             {
//                 v_ref = 0;
//                 omega_ref = 0;
//             }

//             if (opt_max_eval_fun <= 0) // 너무 장애물이 가까워서 최적 경로가 없는 경우
//             {
//                 too_close_flag = true;
//             }
//             if (too_close_flag == true)
//             {
//                 if (hypot(lidar_y[min_index] - position_y, lidar_x[min_index] - position_x) > 1.0)
//                 {
//                     too_close_flag = false;
//                 }

//                 if (abs(robot_theta - atan2(lidar_y[min_index] - position_y, lidar_x[min_index] - position_x)) > DEG2RAD(90))
//                 {
//                     v_ref = 0.2;
//                     omega_ref = (-1.0) * std::copysign(1.0, atan2(lidar_y[min_index] - position_y, lidar_x[min_index] - position_x) - robot_theta) * 0.6;
//                     std::cout << "too close obstacle is behind robot!!" << std::endl;
//                 }
//                 else
//                 {
//                     v_ref = -0.2;
//                     omega_ref = (-1.0) * std::copysign(1.0, atan2(lidar_y[min_index] - position_y, lidar_x[min_index] - position_x) - robot_theta) * 0.3;
//                     std::cout << "too close obstacle is in front of robot!!" << std::endl;
//                 }
//             }

//             v_current = v_ref;
//             omega_current = omega_ref;

//             // cmd_vel 메시지를 발행하는 부분 추가
//             geometry_msgs::msg::Twist cmd_msg;
//             cmd_msg.linear.x = v_ref;
//             cmd_msg.angular.z = omega_ref;
//             cmd_pub->publish(cmd_msg);
//             // std::cout << " [v_ref: " << v_ref << "] / [omega_ref: " << omega_ref << "]" << std::endl;

//             std::cout << std::fixed << std::setprecision(2);
//             // std::cout << "[velocity_score: " << velocity_score << "] , [heading_score: " << heading_score << "], [distance_score: " << distance_score<< "] , [goal_score: " << goal_score << "]"<<std::endl;
//             // std::cout << "[goal_score: " << goal_score << "] , [heading_score: " << heading_score << "], [distance_score: " << distance_score<< "] , [velocity_score: " << velocity_score << "]"<<std::endl;
//         }
//     }

// private:
//     // Variables
//     float goal_x;
//     float goal_y;
//     float position_x = 0;
//     float position_y = 0;
//     float position_accuracy = 0.05;
//     float obstacledistance = 0.7f;

//     float max_distance = 5.0f;              // 로봇 goal까지의 최대 거리
//     float observe_obstacle_distance = 3.0f; // 관측할 장애물 거리, 즉 허용 범위

//     float robot_theta = 0; // 로봇 yaw
//     float *lidar_x;
//     float *lidar_y;
//     float *ranges;

//     float v_ref = 0;
//     float omega_ref = 0;
//     float v_current = 0;
//     float omega_current = 0;

//     bool lidar_flag = false;
//     bool goal_obstacle_check = false;
//     bool too_close_flag = false;
//     // DWA Parameters
//     float a_max = 0.2;
//     float v_max = 0.8;
//     float v_min = 0.05;
//     float omega_max = 0.6 * 3.14;
//     float epsilon_max = 0.2 * 3.14;

//     // STEP
//     float dT = 0.5;
//     float v_accuracy_prediction = 0.005;
//     float omega_accuracy_prediction = 0.001 * 3.14;

//     // Prediction time
//     float prediction_time = 2.0f;

//     int lidar_N = 0;
//     int min_index = 0;
//     bool firstinit = false;

//     // minimum laser range!!!!!
//     float min_range = 0.1;

//     // DWA gain
//     float goal_gain = 0.7f;
//     float velocity_gain = 1.0f;
//     float heading_gain = 0.5f;
//     float obstacle_gain = 1.0f;

//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub;

//     // Add other function and variable declarations as necessary
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<DWANode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }