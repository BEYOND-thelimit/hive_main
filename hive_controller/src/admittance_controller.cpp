#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <Eigen/Dense>
#include <vector>

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

class AdmittanceNode : public rclcpp::Node
{
public:
    AdmittanceNode(const std::string &robot_name)
        : Node("Admittance_planner"), target_x(0.0f), target_y(0.0f),
          position_x(0.0f), position_y(0.0f), robot_theta(0.0f),
          lidar_x(nullptr), lidar_y(nullptr), ranges(nullptr),
          v_ref(0.0f), omega_ref(0.0f),
          v_current(0.0f), omega_current(0.0f),
          firstinit(false), robot_name_(robot_name)
    {
        // 목표 위치 및 파라미터 선언 및 초기화
        this->declare_parameter<float>("target_x", 0.0);
        this->declare_parameter<float>("target_y", 0.0);
        this->get_parameter("target_x", target_x);
        this->get_parameter("target_y", target_y);

        // goal_sub = this->create_subscription<geometry_msgs::msg::Point>(
        //     "/" + robot_name_ + "/goal_point", rclcpp::SensorDataQoS(), std::bind(&AdmittanceNode::goal_callback, this, std::placeholders::_1));

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/" + robot_name_ + "/odom", rclcpp::SensorDataQoS(), std::bind(&AdmittanceNode::odom_callback, this, std::placeholders::_1));

        // "/scan" 토픽에 대한 구독자를 생성합니다. QoS 설정으로 rclcpp::SensorDataQoS()를 사용합니다.
        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/" + robot_name_ + "/converted_scan", rclcpp::SensorDataQoS(), std::bind(&AdmittanceNode::laser_callback, this, std::placeholders::_1));

        // "/cmd_vel" 토픽에 Twist 메시지를 발행하는 발행자를 생성합니다. 발행 큐의 크기는 10으로 설정됩니다.
        cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/" + robot_name_ + "/cmd_vel", 10);
        // goal_status 발행
        goal_status_publisher = this->create_publisher<std_msgs::msg::String>("/" + robot_name_ + "/goal_status", 10);
        // target_position 구독
        target_position_subscription = this->create_subscription<std_msgs::msg::String>(
            "/" + robot_name_ + "/target_position", 10,
            std::bind(&AdmittanceNode::targetPositionCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            100ms, [this]()
            { this->compute(); });

        M.diagonal() << m_gain, m_gain, m_gain, m_gain, m_gain, m_gain;
        D.diagonal() << 2 * d_gain, d_gain, d_gain, d_gain, d_gain, 2 * d_gain;
        K.diagonal() << 8 * k_gain, k_gain, k_gain, k_gain, k_gain, 0.5 * k_gain;
    }

    // Destructor
    ~AdmittanceNode()
    {
        delete[] ranges;
        delete[] lidar_x;
        delete[] lidar_y;
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

        // RCLCPP_INFO(this->get_logger(), "New goal: X=%f, Y=%f", target_x, target_y);
        x_des[0] = target_x;
        x_des[1] = target_y;

        target_yaw = atan2(target_y - position_y, target_x - position_x);
        x_des[5] = target_yaw;

        if (target_x != last_target_x || target_y != last_target_y)
        {
            // last_way_point = way_point;
            target_flag = true;
            goal_arrived_flag = false;
            turn_flag = true;
            last_target_x = target_x;
            last_target_y = target_y;
        }
    }

    // 로봇의 위치와 방향 정보를 업데이트하는 콜백 함수
    void odom_callback(const nav_msgs::msg::Odometry::ConstPtr &msg)
    {
        position_x = msg->pose.pose.position.x;
        position_y = msg->pose.pose.position.y;
        robot_theta = 2.0f * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        linear_velocity_x = msg->twist.twist.linear.x;   // 선속도 x
        angular_velocity_z = msg->twist.twist.angular.z; // 각속도 z
        degree_repair(robot_theta);                      // 각도를 [-π, π] 범위로 정규화

        x_now[0] = position_x;
        x_now[1] = position_y;
        x_now[5] = robot_theta;

        if (init_flag == false)
        {
            x_des[0] = x_now[0];
            x_des[1] = x_now[1];
            x_des[5] = x_now[5];
            init_flag = true;
        }
    }

    // 라이다 데이터를 처리하는 콜백 함수
    void laser_callback(const sensor_msgs::msg::LaserScan::ConstPtr &msg)
    {
        bool check_flag = false;

        lidar_flag = false;
        lidar_N = (int)((msg->angle_max - msg->angle_min) / msg->angle_increment) + 1;

        int count = 1;

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

    // 함수 선언: rad 라디안 만큼 Z축을 중심으로 회전하는 3x3 행렬 반환
    Eigen::Matrix3d createZRotationMatrix(float rad)
    {
        Eigen::Matrix3d rotation_matrix;
        double cos_rad = cos(rad);
        double sin_rad = sin(rad);

        // Z축을 중심으로 회전하는 행렬 설정
        rotation_matrix << cos_rad, -sin_rad, 0,
            sin_rad, cos_rad, 0,
            0, 0, 1;

        return rotation_matrix;
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
        if (std::abs(angle_difference) > 0.035) //2도
        { // 회전해야 할 각도가 임계값(예: 0.01 라디안)보다 큰 경우
            // 선속도를 0으로 설정하여 현재 위치에서 회전만 수행
            cmd_msg.linear.x = 0.0;
            // 각속도 설정, 각도 차이에 따라 각속도를 결정 (여기에서는 단순화된 예시)
            cmd_msg.angular.z = angle_difference > 0 ? 0.3 : -0.3; // 각속도를 적절히 조절하세요
        }
        else
        {
            turn_flag = false;
            RCLCPP_INFO(this->get_logger(), "Turn flag == false.");
            // 목표 방향을 향하고 있을 때의 처리, 필요하다면 선속도를 조절하여 전진하도록 설정
            cmd_msg.linear.x = 0.0; // 필요에 따라 변경
            cmd_msg.angular.z = 0.0;
        }
        std::cout << "turning!!" << std::endl;
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

    void compute()
    {
        goalObstacleCheckFunc();
        if (turn_flag == true)
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

            // 30도보다 큰 각도 차이가 있는 경우에만 방향 조정
            if (std::abs(angle_difference) > DEG2RAD(turn_degree)) // 30도를 라디안으로 변환
            {
                rotateTowardsTarget(robot_theta, position_x, position_y, target_x, target_y);
                // RCLCPP_INFO(this->get_logger(), "turning");
            }
            else
            {
                if(turn_flag == true)///////////////////////////////////////방금 추가한 부분
                {
                    x_dot = Eigen::VectorXd::Zero(6);
                    std::cout << "x_dot reset!!" << std::endl;
                }
                turn_flag = false; // 필요한 경우 플래그 업데이트
                // RCLCPP_INFO(this->get_logger(), "minor angle difference, not turning");
            }
        }
        else if (goal_arrived_flag == false && target_flag == true && turn_flag == false && goal_obstacle_check == false)
        {
            if (hypot(target_x - position_x, target_y - position_y) <= position_accuracy)
            {

                goal_arrived_flag = true;
            }
            else
            {
                goal_arrived_flag = false;
            }

            if (lidar_flag == true)
            {
                float force_degree = atan2(position_y - lidar_y[min_index], position_x - lidar_x[min_index]);
                float force_size = hypot(position_y - lidar_y[min_index], position_x - lidar_x[min_index]);
                if (force_size < too_close_distance)
                {
                    float obstacle_degree = atan2(lidar_y[min_index] - position_y, lidar_x[min_index] - position_x) - robot_theta;
                    if (obstacle_degree > DEG2RAD(0) && obstacle_degree < DEG2RAD(90))
                    {
                        force[5] = 0.5;
                    }
                    else if (obstacle_degree > DEG2RAD(90) && obstacle_degree < DEG2RAD(180))
                    {
                        force[5] = -0.5;
                    }
                    else if (obstacle_degree < DEG2RAD(0) && obstacle_degree > DEG2RAD(-90))
                    {
                        force[5] = -0.5;
                    }
                    else if (obstacle_degree < DEG2RAD(-90) && obstacle_degree > DEG2RAD(-180))
                    {
                        force[5] = 0.5;
                    }
                    else
                    {
                        force[5] = 0.0;
                    }
                    float min_value = 0.0;                // force_size의 최소값
                    float max_value = too_close_distance; // force_size의 최대값
                    float max_force = 3.0;                // 매핑하고 싶은 최대 힘의 값
                    // force_size 값을 0에서 max_force 사이로 매핑
                    float mapped_force_size = (force_size - min_value) / (max_value - min_value) * max_force;
                    // 결과 값이 0 미만이면 0으로, max_force 초과면 max_force로 제한;
                    force_size = max_force - std::max(0.0f, std::min(mapped_force_size, max_force));
                }
                else
                {
                    force_size = 0.0;
                }
                Eigen::Matrix3d rotation_matrix = createZRotationMatrix(robot_theta); // 회전 행렬 생성
                force[0] = force_size * cos(force_degree);
                force[1] = force_size * sin(force_degree);
                force.head(3) = rotation_matrix.transpose() * force.head(3);

                x_err = x_des - x_now;
                x_err.head(3) = rotation_matrix.transpose() * x_err.head(3); //로봇프레임 기준 어디에 있는지
                x_err[5] = atan2(x_err[1],x_err[0]); //로봇 프레임 기준 얼마나 돌아갔는지
                if (x_err[0] <= 0)
                {
                    if (x_err[5] >= DEG2RAD(90) && x_err[5] < DEG2RAD(180))
                    {
                        x_err[5] = x_err[5] - DEG2RAD(180);
                    }
                    else if (x_err[5] <= -1 * DEG2RAD(90) && x_err[5] > -1 * DEG2RAD(180))
                    {
                        x_err[5] = x_err[5] + DEG2RAD(180);
                    }
                    else
                    {
                        x_err[5] = 0.0;
                    }
                }

                // std::cout << "[target: " << x_des[0] << ", "<<x_des[1] << "], [" << x_err.transpose() << "]" << std::endl;
                



                x_ddot = M.inverse() * K * (x_err - D * x_dot + force);
                x_dot = x_dot + x_ddot * dt;
                std::cout << x_dot.transpose() << std::endl;

                geometry_msgs::msg::Twist cmd_msg;
                if (abs(x_dot[0]) > v_max)
                    cmd_msg.linear.x = std::copysign(1.0, x_dot[0]) * v_max;
                else
                    cmd_msg.linear.x = x_dot[0];
                if (abs(x_dot[5]) > omega_max)
                    cmd_msg.angular.z = std::copysign(1.0, x_dot[5]) * omega_max;
                else
                    cmd_msg.angular.z = x_dot[5];


                // // 선형 속도의 급변동 방지
                // if (std::abs(cmd_msg.linear.x - prev_linear_x) / dt > max_acceleration) {
                //     cmd_msg.linear.x = prev_linear_x + std::copysign(max_acceleration * dt, cmd_msg.linear.x - prev_linear_x);
                // }

                // // 각속도의 급변동 방지
                // if (std::abs(cmd_msg.angular.z - prev_angular_z) / dt > max_angular_acceleration) {
                //     cmd_msg.angular.z = prev_angular_z + std::copysign(max_angular_acceleration * dt, cmd_msg.angular.z - prev_angular_z);
                // }

                // // 속도 제한
                // cmd_msg.linear.x = std::max(std::min(float(cmd_msg.linear.x), v_max), -v_max);
                // cmd_msg.angular.z = std::max(std::min(float(cmd_msg.angular.z), omega_max), -omega_max);
                cmd_pub->publish(cmd_msg);
                prev_linear_x = cmd_msg.linear.x;
                prev_angular_z = cmd_msg.angular.z;
            }
            else
            {
                Eigen::Matrix3d rotation_matrix = createZRotationMatrix(robot_theta); // 회전 행렬 생성
                x_err = x_des - x_now;
                x_err.head(3) = rotation_matrix.transpose() * x_err.head(3); //로봇프레임 기준 어디에 있는지
                x_err[5] = atan2(x_err[1],x_err[0]); //로봇 프레임 기준 얼마나 돌아갔는지

                if (x_err[0] <= 0)
                {
                    if (x_err[5] >= DEG2RAD(90) && x_err[5] < DEG2RAD(180))
                    {
                        x_err[5] = x_err[5] - DEG2RAD(180);
                    }
                    else if (x_err[5] <= -1 * DEG2RAD(90) && x_err[5] > -1 * DEG2RAD(180))
                    {
                        x_err[5] = x_err[5] + DEG2RAD(180);
                    }
                    else
                    {
                        x_err[5] = 0.0;
                    }
                }

                // std::cout << "[target: " << x_des[0] << ", "<<x_des[1] << "], [" << x_err.transpose() << "]" << std::endl;


                if(abs(x_err[5]>5.0))
                {
                    std::cout << "x_des_yaw: " << x_des[5] << " / x_now_yaw: " << x_now[5] << std::endl;
                }

                x_ddot = M.inverse() * (K * x_err - D * x_dot);
                x_dot = x_dot + x_ddot * dt;
                std::cout << x_dot.transpose() << std::endl;

                geometry_msgs::msg::Twist cmd_msg;
                if (abs(x_dot[0]) > v_max)
                    cmd_msg.linear.x = std::copysign(1.0, x_dot[0]) * v_max;
                else
                    cmd_msg.linear.x = x_dot[0];
                if (abs(x_dot[5]) > omega_max)
                    cmd_msg.angular.z = std::copysign(1.0, x_dot[0]) * omega_max;
                else
                    cmd_msg.angular.z = x_dot[5];

                // // 선형 속도의 급변동 방지
                // if (std::abs(cmd_msg.linear.x - prev_linear_x) / dt > max_acceleration) {
                //     cmd_msg.linear.x = prev_linear_x + std::copysign(max_acceleration * dt, cmd_msg.linear.x - prev_linear_x);
                // }

                // // 각속도의 급변동 방지
                // if (std::abs(cmd_msg.angular.z - prev_angular_z) / dt > max_angular_acceleration) {
                //     cmd_msg.angular.z = prev_angular_z + std::copysign(max_angular_acceleration * dt, cmd_msg.angular.z - prev_angular_z);
                // }

                // // 속도 제한
                // cmd_msg.linear.x = std::max(std::min(float(cmd_msg.linear.x), v_max), -v_max);
                // cmd_msg.angular.z = std::max(std::min(float(cmd_msg.angular.z), omega_max), -omega_max);
                cmd_pub->publish(cmd_msg);
                prev_linear_x = cmd_msg.linear.x;
                prev_angular_z = cmd_msg.angular.z;
                // std::cout << "desire linear vel x: " << cmd_msg.linear.x << " / desire angular vel z: " << cmd_msg.angular.z << std::endl;
            }
        }
        // 도착함을 publish
        else if (goal_arrived_flag == true)
        {
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
            // turn_flag == true;
            // 목표에 도달했으므로 로봇을 멈춤
            geometry_msgs::msg::Twist stop_msg;
            stop_msg.linear.x = 0.0;
            stop_msg.angular.z = 0.0;
            cmd_pub->publish(stop_msg);

            RCLCPP_INFO(this->get_logger(), "Goal reached, stopping.");
            // std::cout << "way_point: " << way_point << std::endl;

            target_flag = false;
        }
    }

private:
    // Variables
    float target_x;
    float target_y;
    float last_target_x = -123.0f;
    float last_target_y = -123.0f;
    float position_x = 0;
    float position_y = 0;
    float linear_velocity_x = 0.0;
    float angular_velocity_z = 0.0;
    int turn_degree = 18;

    float angle_difference = 0.0;
    float position_accuracy = 0.00022;

    bool turn_flag = true;            // 돌고 출발할거 확인용, 돌아야 하면 true
    bool target_flag = false;         // 이전 target과 같은지 보는것.즉, 이전 target과 다르면 움직여야 함 -> true
    bool goal_arrived_flag = false;   // 도착 했는지 체크
    bool goal_obstacle_check = false; // 최종 목적지에 장애물 있는지 체크
    int way_point = 0;
    bool last_arrived = false; // 최종 골에 도달 했는지

    float robot_theta = 0; // 로봇 yaw
    float target_yaw = 0.0;
    float last_target_yaw = 0.0;
    float *lidar_x;
    float *lidar_y;
    float *ranges;

    float v_ref = 0;
    float omega_ref = 0;
    float v_current = 0;
    float omega_current = 0;

    bool lidar_flag = false;
    bool too_close_flag = false;

    float v_max = 0.22;
    float omega_max = 0.4 * 3.14;
    float prev_linear_x = 0.0;
    float prev_angular_z = 0.0;
    float max_acceleration = 0.05;  // 최대 가속도 (m/s^2)
    float max_deceleration = 0.05;  // 최대 감속도 (m/s^2)
    float max_angular_acceleration = 0.1 * 3.14;  // 최대 각 가속도 (rad/s^2)


    int lidar_N = 0;
    int min_index = 0;
    bool firstinit = false;

    // minimum laser range!!!!!
    float min_range = 0.1;

    // admittance parameter
    float too_close_distance = 1.0;
    float dt = 0.1;
    float m_gain = 1.0;
    float d_gain = 1.0;
    float k_gain = 1.0;

    Eigen::MatrixXd M = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd D = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd K = Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd x_ddot = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd x_dot = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd x_now = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd x_des = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd x_err = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd force = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd des_cmd_vel = Eigen::VectorXd::Zero(6);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_position_subscription;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_status_publisher;

    // Add other function and variable declarations as necessary
    bool init_flag = false;
    std::string robot_name_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string robot_name = "robot1"; // "robot2", "robot3" 등으로 변경
    auto node = std::make_shared<AdmittanceNode>(robot_name);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
