#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std;

#define DEG2RAD(a) ((a) * (M_PI / 180.0f))

class RobotController : public rclcpp::Node
{
public:
    RobotController()
        : Node("p_controller"), // 노드 이름에 로봇 이름을 포함시킵니다.
          initialized_(false),
          current_goal_idx_(0), // 현재 목표 인덱스 초기화
          curr_target_index_(0), // 현재 경로 인덱스 초기화
          kp_(0.1), ki_(0.0), kd_(0.3), prev_error_(0.0), integral_(0.0)
    {
        // 로봇 이름을 사용하여 토픽 이름을 동적으로 설정합니다.
        // std::string odom_topic = "/" + robot_name_ + "/odom";
        // std::string cmd_vel_topic = "/" + robot_name_ + "/cmd_vel";
        // std::string target_position_topic = "/" + robot_name_ + "/target_position";

        // odom 토픽을 구독합니다.
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&RobotController::odom_callback, this, _1));

        // target_position 토픽을 구독합니다.
        target_position_subscription = this->create_subscription<std_msgs::msg::String>(
            "target_position", 10, std::bind(&RobotController::targetPositionCallback, this, _1));

        // target_position 토픽을 구독합니다.
        dockingPostion_subscription = this->create_subscription<std_msgs::msg::String>(
            "docking_position", 10, std::bind(&RobotController::dockingPositionCallback, this, _1));

        // cmd_vel 토픽에 명령어를 발행합니다.
        cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // "/scan" 토픽에 대한 구독자를 생성합니다. QoS 설정으로 rclcpp::SensorDataQoS()를 사용합니다.
        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "converted_scan", rclcpp::SensorDataQoS(), std::bind(&RobotController::laser_callback, this, std::placeholders::_1));

        // goal_status 발행
        goal_status_publisher = this->create_publisher<std_msgs::msg::String>("/goal_status", 10);

        // 100ms마다 control 함수를 호출하는 타이머 설정
        timer_ = this->create_wall_timer(
            100ms, std::bind(&RobotController::compute, this));

        // 로봇의 초기 위치 및 목표 위치를 설정합니다.
        curr_pose_ = {0.0, 0.0};
        target_pose_ = {0.0, 0.0};
        curr_ori_ = geometry_msgs::msg::Quaternion();
        position_error_tolerance = 0.0;//0.0044 * 4 * 0.05; // 한 그리드가 0.0044 * 4이므로 에러 10프로 이내이면
        robot_theta = 0.0;

        // 초기 경로 생성
        // generateBezierTrajectory(); // 초기에는 경로를 생성하지 않음
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_position_subscription;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dockingPostion_subscription;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_status_publisher;
    rclcpp::TimerBase::SharedPtr timer_; // 타이머

    ///////////////////////////////////////////////////////////////////////parameter/////////////////////////////////////////////////////////////////////////////////////
    std::string robot_name_;                  // 로봇 이름
    std::array<float, 2> curr_pose_;          // 현재 위치
    geometry_msgs::msg::Quaternion curr_ori_; // 현재 방향
    std::array<float, 2> target_pose_;        // 목표 x,y;
    float target_yaw = 0.0;                   // 목표 yaw(방향)
    bool initialized_;                        // 초기화 여부
    double position_error_tolerance;          // position error 허용거리 m

    float last_target_x = -123.0f;
    float last_target_y = -123.0f;
    int way_point = 0;

    bool last_arrived = false;        // 최종 골에 도달 했는지
    bool target_flag = false;         // 이전 target과 같은지 보는것.즉, 이전 target과 다르면 움직여야 함 -> true
    bool goal_arrived_flag = false;   // 도착 했는지 체크
    bool turn_flag = true;            // 돌고 출발할거 확인용, 돌아야 하면 true
    bool goal_obstacle_check = false; // 최종 목적지에 장애물 있는지 체크

    float robot_theta; // 현재 로봇 yaw(rad)
    float linear_velocity_x = 0.0;
    float angular_velocity_z = 0.0;

    // lidar parameter
    bool lidar_flag = false;
    int lidar_N = 0;
    bool firstinit = false;
    float *lidar_x;
    float *lidar_y;
    float *ranges;
    float min_range = 0.1;
    int min_index = 0;
    float angle_difference = 0.0;
    float too_close_distance = 0.5; // 밀려나야 할 거리
    float REAL_X = 2.88;
    float REAL_Y = 2.24;//m
    float fixelConst_x = REAL_X/(640/4); //0.018 m
    float fixelConst_y = REAL_Y/(480/4); //m

    Eigen::VectorXd x_err = Eigen::VectorXd::Zero(3);

    float max_linear_vel = 0.25; // m/s
    float max_angular_vel = 0.5; // rad/s

    float start_turn_degree = 3;
    float linear_vel_gain = 1.0;
    float angular_vel_gain = 0.2;

    std::vector<std::pair<float, float>> trajectory_; // 베지어 경로 저장
    float kp_, ki_, kd_;
    float prev_error_, integral_;
    size_t curr_target_index_;
    size_t current_goal_idx_; // 현재 목표 인덱스
    bool trajec_flag = false;
    float v = 0;
    float v_yaw = 0;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

    // target_position 토픽의 콜백 함수입니다.
    void targetPositionCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::vector<std::string> parsed_data = split(msg->data, ',');
        if (parsed_data.size() < 4)
            return; // 데이터 검증

        target_pose_[0] = std::stod(parsed_data[0]);
        target_pose_[1] = std::stod(parsed_data[1]);
        way_point = std::stoi(parsed_data[2]);
        last_arrived = std::stoi(parsed_data[3]) != 0; // 도착 여부, 0이 아니면 true

        target_yaw = atan2(target_pose_[1] - curr_pose_[1], target_pose_[0] - curr_pose_[0]);
        if (target_pose_[0] != last_target_x || target_pose_[1] != last_target_y)
        {
            // last_way_point = way_point;
            target_flag = true;
            goal_arrived_flag = false;
            turn_flag = true;
            last_target_x = target_pose_[0];
            last_target_y = target_pose_[1];
        }
    }

    void dockingPositionCallback(const std_msgs::msg::String::ConstPtr msg)
    {
        double dockingPose_[3] = {0,0,0};
        vector<string> dockingPose_data = split(msg->data, ',');
        if (dockingPose_data.size() < 3)
            return;


        dockingPose_[0] = stod(dockingPose_data[0]);
        dockingPose_[1] = stod(dockingPose_data[1]);
        dockingPose_[2] = stod(dockingPose_data[2]);

        trajec_flag = true;
        target_flag = true;
        generateBezierTrajectory(dockingPose_[0], dockingPose_[1], dockingPose_[2]);

        std::cout << "docking Callback DONE!!" << std::endl;
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
                lidar_x[i] = curr_pose_[0] + xl * cos(robot_theta) - yl * sin(robot_theta);
                lidar_y[i] = curr_pose_[1] + xl * sin(robot_theta) + yl * cos(robot_theta);
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

    // 로봇의 위치와 방향 정보를 업데이트하는 콜백 함수
    void odom_callback(const nav_msgs::msg::Odometry::ConstPtr &msg)
    {
        curr_pose_[0] = msg->pose.pose.position.x;
        curr_pose_[1] = msg->pose.pose.position.y;
        robot_theta = 2.0f * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        linear_velocity_x = msg->twist.twist.linear.x;   // 선속도 x
        angular_velocity_z = msg->twist.twist.angular.z; // 각속도 z
        degree_repair(robot_theta);                      // 각도를 [-π, π] 범위로 정규화
    }

    // 쿼터니언을 오일러 각도로 변환합니다.
    double quaternion_to_euler(const geometry_msgs::msg::Quaternion &q)
    {
        double roll, pitch, yaw;
        std::tie(roll, pitch, yaw) = euler_from_quaternion(q);
        return yaw;
    }

    // 주어진 쿼터니언을 오일러 각도로 변환합니다.
    std::tuple<double, double, double> euler_from_quaternion(const geometry_msgs::msg::Quaternion &q)
    {
        double x = q.x;
        double y = q.y;
        double z = q.z;
        double w = q.w;

        double t0 = +2.0 * (w * x + y * z);
        double t1 = +1.0 - 2.0 * (x * x + y * y);
        double roll = std::atan2(t0, t1);

        double t2 = +2.0 * (w * y - z * x);
        t2 = t2 > +1.0 ? +1.0 : t2;
        t2 = t2 < -1.0 ? -1.0 : t2;
        double pitch = std::asin(t2);

        double t3 = +2.0 * (w * z + x * y);
        double t4 = +1.0 - 2.0 * (y * y + z * z);
        double yaw = std::atan2(t3, t4);

        return std::make_tuple(roll, pitch, yaw);
    }

    // 속도의 최대 값을 제한하는 함수입니다.
    double checkLinearLimitVelocity(float velocity, float max_velocity = 0.25)
    {
        return std::max(std::min(velocity, max_velocity), -max_velocity);
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
        if (std::abs(angle_difference) > 0.035) // 2도
        {                                       // 회전해야 할 각도가 임계값(예: 0.01 라디안)보다 큰 경우
            // 선속도를 0으로 설정하여 현재 위치에서 회전만 수행
            cmd_msg.linear.x = 0.0;
            // 각속도 설정, 각도 차이에 따라 각속도를 결정 (여기에서는 단순화된 예시)
            cmd_msg.angular.z = angle_difference > 0 ? 0.15 : -0.15; // 각속도를 적절히 조절하세요
        }
        else
        {
            turn_flag = false;
            RCLCPP_INFO(this->get_logger(), "Turn flag == false.");
            // 목표 방향을 향하고 있을 때의 처리, 필요하다면 선속도를 조절하여 전진하도록 설정
            cmd_msg.linear.x = 0.0; // 필요에 따라 변경
            cmd_msg.angular.z = 0.0;
        }
        // std::cout << "turning!!" << std::endl;
        cmd_pub->publish(cmd_msg);
    }

    void goalObstacleCheckFunc()
    {
        // goal 위치에 장애물이 있다면 대기. 로봇이 goal에 대해 10도 이내로 바라보고, 거리가 0.9미터 이내라면 장애물이 goal에 있다고 판별
        if (lidar_flag == true && abs(robot_theta - atan2(lidar_y[min_index] - curr_pose_[1], lidar_x[min_index] - curr_pose_[0])) < DEG2RAD(10) && abs(robot_theta - atan2(target_pose_[1] - curr_pose_[1], target_pose_[0] - curr_pose_[0])) < DEG2RAD(10) && hypot(lidar_x[min_index] - curr_pose_[0], lidar_y[min_index] - curr_pose_[1]) < 0.9 && hypot(lidar_x[min_index] - curr_pose_[0], lidar_y[min_index] - curr_pose_[1]) < 0.9 && hypot(target_pose_[0] - curr_pose_[0], target_pose_[1] - curr_pose_[1]) < 1.0)
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
            // std::cout << "pub 1" << std::endl;

            cmd_pub->publish(cmd_msg);
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

    // 베지어 곡선을 계산하는 함수
    std::pair<float, float> bezierCurve(const Eigen::Vector2f &P0, const Eigen::Vector2f &P1, const Eigen::Vector2f &P2, const Eigen::Vector2f &P3, float t)
    {
        float u = 1 - t;
        float tt = t * t;
        float uu = u * u;
        float uuu = uu * u;
        float ttt = tt * t;

        Eigen::Vector2f point = uuu * P0; // (1-t)^3 * P0
        point += 3 * uu * t * P1;         // 3(1-t)^2 * t * P1
        point += 3 * u * tt * P2;         // 3(1-t) * t^2 * P2
        point += ttt * P3;                // t^3 * P3

        return {point.x(), point.y()}; // Eigen::Vector2f를 std::pair<float, float>로 변환하여 반환
    }
    // yaw 각도(라디안)를 고려하여 P2를 조정하는 함수
    Eigen::Vector2f adjustP2ForYaw(const Eigen::Vector2f &P3, float yaw, float distance)
    {
        float deltaX = std::cos(yaw) * distance; // x 방향 변화량
        float deltaY = std::sin(yaw) * distance; // y 방향 변화량
        return Eigen::Vector2f(P3.x() - deltaX, P3.y() - deltaY);
    }

    // 베지어 곡선을 따라 경로를 생성하는 함수
    void generateBezierTrajectory(float x, float y, float yaw)
    {                        // yaw 파라미터 추가
        trajectory_.clear(); // 기존 경로 초기화

        //!!!!!!!!!실제에서는 아래 X,Y 바뀐거 사용해야함.
        // x = y * fixelConst_y; //좌표계 뒤집어져있어서 반대로 넣음.
        // y = x * fixelConst_x;

        x = x * fixelConst_x;
        y = y * fixelConst_y; //좌표계 뒤집어져있어서 반대로 넣음.

        
        Eigen::Vector2f P0(curr_pose_[0], curr_pose_[1]);
        Eigen::Vector2f P1((curr_pose_[0] + x) / 2, curr_pose_[1]);

        Eigen::Vector2f P3(x, y);
        // P2를 조정하기 위해 yaw를 고려
        float distance = 0.1; // P3에서 P2까지의 거리, 적절한 값을 선택해야 함
        Eigen::Vector2f P2 = adjustP2ForYaw(P3, yaw, distance);

        float step_size = 0.2f; // 스텝 크기 설정

        for (float t = 0; t < 1.0f; t += step_size)
        {
            trajectory_.emplace_back(bezierCurve(P0, P1, P2, P3, t));
        }
        trajectory_.emplace_back(P3.x(), P3.y()); // 마지막 점을 추가

        // 생성된 경로를 출력
        for (const auto &point : trajectory_)
        {
            std::cout << "Point: (" << point.first << ", " << point.second << ")" << std::endl;
            std::cout << "Gird Point: (" << point.first/fixelConst_x << ", " << point.second/fixelConst_y << ")" << std::endl;

        }
    }

    // void trajectoryFollowFunction(float dt)
    // {

    //     if (curr_target_index_ >= trajectory_.size())
    //     {

    //         RCLCPP_INFO(this->get_logger(), "Reached the end of the trajectory.");
    //         goal_arrived_flag = true;
    //         trajec_flag = false;
    //         return;
    //     }

    //     const auto &point = trajectory_[curr_target_index_];

    //     float distance_error = sqrt(pow(point.first - curr_pose_[0], 2) + pow(point.second - curr_pose_[1], 2));
    //     float angle_to_target = atan2(point.second - curr_pose_[1], point.first - curr_pose_[0]);
    //     float yaw_error = angle_to_target - robot_theta;


    //     // 선형 속도 제어
    //     float v_desired = control(distance_error, dt);


    //     // 최대 가속도 제한
    //     float max_acceleration = 0.02; // 최대 가속도
    //     float acceleration = (v_desired - linear_velocity_x) / dt;
    //     if (acceleration > max_acceleration)
    //     {
    //         acceleration = max_acceleration;
    //     }
    //     else if (acceleration < -max_acceleration)
    //     {
    //         acceleration = -max_acceleration;
    //     }

    //     // 다음 속도 계산
    //     linear_velocity_x += acceleration * dt;


    //     // 각속도 제어
    //     float angular_velocity = control(yaw_error, dt);


    //     // ROS2 메시지로 변환
    //     auto cmd_vel = geometry_msgs::msg::Twist();
    //     cmd_vel.linear.x = linear_velocity_x;
    //     cmd_vel.angular.z = angular_velocity;

    //     // cmd_vel 토픽으로 발행
    //     cmd_pub->publish(cmd_vel);

    //     // 목표 점에 도달했는지 체크
    //     if (distance_error < position_error_tolerance)
    //     {
    //         curr_target_index_++;
    //     }

    //     // 출력
    //     std::cout << "Next position: (" << curr_pose_[0] << ", " << curr_pose_[1] << ", Next speed: " << linear_velocity_x << "), Next yaw: " << robot_theta  << std::endl;
    // }

    void trajectoryFollowFunction(float dt)
    {
        auto cmd_vel = geometry_msgs::msg::Twist();
        if (curr_target_index_ >= trajectory_.size())
        {

            RCLCPP_INFO(this->get_logger(), "Reached the end of the trajectory.");
            goal_arrived_flag = true;
            trajec_flag = false;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_pub->publish(cmd_vel); // 명령어를 발행합니다.
            return;
        }
        if(curr_target_index_ == 0)curr_target_index_++;
        const auto &point = trajectory_[curr_target_index_];

        x_err(0) = point.first - curr_pose_[0];
        x_err(1) = point.second - curr_pose_[1];
        float distance_error = std::sqrt(float(x_err(0)) * float(x_err(0)) + float(x_err(1)) * float(x_err(1))); // 목표까지의 거리를 계산
        Eigen::VectorXd robot_x_err = Eigen::VectorXd::Zero(3);                                                  // 로봇 프레임 기준 pose err
        Eigen::Matrix3d rotation_matrix = createZRotationMatrix(robot_theta);                                    // 회전 행렬 생성
        robot_x_err = rotation_matrix.transpose() * x_err;
        float robot_target_angle = atan2(float(robot_x_err(1)), float(robot_x_err(0))); // 로봇 프레임 기준 타겟까지 각도
        // std::cout << "[distance_error: " << distance_error<< ", rotation_error: " << robot_target_angle << "]" << std::endl;
        float docking_linear_vel_gain = 0.65;
        float docking_angular_vel_gain = 0.24;

        if (robot_x_err(0) >= 0)
        {
            if (distance_error > position_error_tolerance)
            {
                if(abs(robot_target_angle) > abs(DEG2RAD(start_turn_degree)))
                {

                    v = 0.0;
                    v_yaw = checkLinearLimitVelocity(docking_angular_vel_gain * robot_target_angle, max_angular_vel); // 목표 방향으로의 회전을 조금 적용
                }
                else
                {   

                    v = checkLinearLimitVelocity(docking_linear_vel_gain * distance_error, max_linear_vel);    // 속도를 거리 오차에 비례하게 조절
                    v_yaw = checkLinearLimitVelocity(docking_angular_vel_gain * robot_target_angle, max_angular_vel); // 목표 방향으로의 회전을 조금 적용
                }
            }
            else
            {

                v = 0.0;                                                                     // 위치에 도달했으므로 전진 속도를 0으로 설정
                v_yaw = checkLinearLimitVelocity(docking_angular_vel_gain * robot_target_angle, max_angular_vel); // 목표 yaw를 정확히 맞추기 위해 회전 속도를 조절

                if (std::abs(robot_target_angle) < 0.05)
                {

                    v_yaw = 0.0;
                    RCLCPP_INFO(this->get_logger(), "Yaw aligned. Arrived at the target position and orientation.");
                }
            }
        }

        
        cmd_vel.linear.x = v;
        cmd_vel.angular.z = v_yaw;
        cmd_pub->publish(cmd_vel); // 명령어를 발행합니다.

        // 목표 점에 도달했는지 체크
        if (distance_error < position_error_tolerance)
        {
            curr_target_index_++;
        }

        // 출력
        // std::cout <<"i "<< curr_target_index_<< ", Next target: (" << point.first << ", " << point.second << ", error: " << distance_error << ", "<< robot_target_angle << ")"  << std::endl;
    }


    float control(float error, float dt)
    {
        integral_ += error * dt;
        float derivative = (error - prev_error_) / dt;
        float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        prev_error_ = error;
        return output;
    }

    void compute()
    {

        goalObstacleCheckFunc();

        // 목표가 설정되지 않았으면 아무것도 하지 않음
        if (!target_flag) 
        {
            return;
        }
        if (trajec_flag == true)
        {
            trajectoryFollowFunction(0.1);
        }
        else
        {
            if (turn_flag == true)
            {
                // 목표 방향 계산
                float target_direction = atan2(target_pose_[1] - curr_pose_[1], target_pose_[0] - curr_pose_[0]);
                // 현재 방향과 목표 방향 사이의 각도 차이 계산
                float angle_difference = target_direction - robot_theta;

                // 각도 차이를 -π ~ π 범위로 정규화
                while (angle_difference > M_PI)
                    angle_difference -= 2 * M_PI;
                while (angle_difference < -M_PI)
                    angle_difference += 2 * M_PI;

                // 원하는 각도보다 큰 각도 차이가 있는 경우에만 방향 조정
                if (std::abs(angle_difference) > DEG2RAD(start_turn_degree)) // 원하는 각도를 라디안으로 변환
                {
                    rotateTowardsTarget(robot_theta, curr_pose_[0], curr_pose_[1], target_pose_[0], target_pose_[1]);
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
                if (hypot(target_pose_[0] - curr_pose_[0], target_pose_[1] - curr_pose_[1]) <= position_error_tolerance)
                {
                    goal_arrived_flag = true;
                }
                else
                {
                    goal_arrived_flag = false;
                }

                if (lidar_flag == true && goal_arrived_flag == false)
                {
                    float obstacle_distance = hypot(curr_pose_[1] - lidar_y[min_index], curr_pose_[0] - lidar_x[min_index]);
                    if (obstacle_distance < too_close_distance)
                    {
                        float obstacle_degree = atan2(lidar_y[min_index] - curr_pose_[1], lidar_x[min_index] - curr_pose_[0]) - robot_theta;
                        if (obstacle_degree > DEG2RAD(0) && obstacle_degree < DEG2RAD(90))
                        {
                            v = -0.1;
                            v_yaw = 0.2;
                        }
                        else if (obstacle_degree > DEG2RAD(90) && obstacle_degree < DEG2RAD(180))
                        {
                            v = 0.1;
                            v_yaw = -0.2;
                        }
                        else if (obstacle_degree < DEG2RAD(0) && obstacle_degree > DEG2RAD(-90))
                        {
                            v = -0.1;
                            v_yaw = -0.2;
                        }
                        else if (obstacle_degree < DEG2RAD(-90) && obstacle_degree > DEG2RAD(-180))
                        {
                            v = 0.1;
                            v_yaw = 0.2;
                        }
                        else
                        {
                            v = -0.1;
                            v_yaw = 0.0;
                        }
                        auto cmd_vel = geometry_msgs::msg::Twist();
                        cmd_vel.linear.x = v;
                        cmd_vel.angular.z = v_yaw;
                        // std::cout << "pub 2" << std::endl;

                        cmd_pub->publish(cmd_vel); // 명령어를 발행합니다.
                    }
                    else
                    {
                        x_err(0) = target_pose_[0] - curr_pose_[0];
                        x_err(1) = target_pose_[1] - curr_pose_[1];
                        float distance_error = std::sqrt(float(x_err(0)) * float(x_err(0)) + float(x_err(1)) * float(x_err(1))); // 목표까지의 거리를 계산
                        Eigen::VectorXd robot_x_err = Eigen::VectorXd::Zero(3);                                                  // 로봇 프레임 기준 pose err
                        Eigen::Matrix3d rotation_matrix = createZRotationMatrix(robot_theta);                                    // 회전 행렬 생성
                        robot_x_err = rotation_matrix.transpose() * x_err;
                        float robot_target_angle = atan2(float(robot_x_err(1)), float(robot_x_err(0))); // 로봇 프레임 기준 타겟까지 각도
                        // std::cout << "[distance_error: " << distance_error<< ", rotation_error: " << robot_target_angle << "]" << std::endl;

                        if (robot_x_err(0) >= 0)
                        {
                            if (distance_error > position_error_tolerance)
                            {
                                // std::cout << "err 1" << std::endl;
                                v = checkLinearLimitVelocity(linear_vel_gain * distance_error + 0.1, max_linear_vel);     // 속도를 거리 오차에 비례하게 조절
                                v_yaw = checkLinearLimitVelocity(angular_vel_gain * robot_target_angle, max_angular_vel); // 목표 방향으로의 회전을 조금 적용
                            }
                            else
                            {
                                // std::cout << "err 2" << std::endl;

                                v = 0.0;                                                                                  // 위치에 도달했으므로 전진 속도를 0으로 설정
                                v_yaw = checkLinearLimitVelocity(angular_vel_gain * robot_target_angle, max_angular_vel); // 목표 yaw를 정확히 맞추기 위해 회전 속도를 조절

                                if (std::abs(robot_target_angle) < 0.05)
                                {
                                    v_yaw = 0.0;
                                    RCLCPP_INFO(this->get_logger(), "Yaw aligned. Arrived at the target position and orientation.");
                                }
                            }
                        }
                        // else
                        // {
                        //     if (distance_error > position_error_tolerance)
                        //     {
                        //         if (robot_target_angle > 0)
                        //         {
                        //             std::cout << "err 3" << std::endl;

                        //             v = checkLinearLimitVelocity(-0.1 * linear_vel_gain * distance_error, max_linear_vel);             // 속도를 거리 오차에 비례하게 조절
                        //             v_yaw = checkLinearLimitVelocity(angular_vel_gain * (robot_target_angle - M_PI), max_angular_vel); // 목표 방향으로의 회전을 조금 적용
                        //         }
                        //         else if (robot_target_angle < 0)
                        //         {
                        //             std::cout << "err 4" << std::endl;

                        //             v = checkLinearLimitVelocity(-0.1 * linear_vel_gain * distance_error, max_linear_vel);             // 속도를 거리 오차에 비례하게 조절
                        //             v_yaw = checkLinearLimitVelocity(angular_vel_gain * (robot_target_angle + M_PI), max_angular_vel); // 목표 방향으로의 회전을 조금 적용
                        //         }
                        //         else
                        //         {
                        //             std::cout << "err 5" << std::endl;

                        //             v = checkLinearLimitVelocity(-0.1 * linear_vel_gain * distance_error, max_linear_vel); // 속도를 거리 오차에 비례하게 조절
                        //             v_yaw = 0.0;
                        //         }
                        //     }
                        //     else
                        //     {
                        //         std::cout << "err 6" << std::endl;

                        //         v = 0.0;                                                                                           // 위치에 도달했으므로 전진 속도를 0으로 설정
                        //         v_yaw = checkLinearLimitVelocity(angular_vel_gain * (robot_target_angle + M_PI), max_angular_vel); // 목표 yaw를 정확히 맞추기 위해 회전 속도를 조절

                        //         if (std::abs(robot_target_angle) < 0.05)
                        //         {
                        //             v_yaw = 0.0;
                        //             RCLCPP_INFO(this->get_logger(), "Yaw aligned. Arrived at the target position and orientation.");
                        //         }
                        //     }
                        // }
                        // std::cout << "[v: " << v << ", v_yaw: " << v_yaw << "]" << std::endl;

                        auto cmd_vel = geometry_msgs::msg::Twist();
                        cmd_vel.linear.x = v;
                        cmd_vel.angular.z = v_yaw;
                        // std::cout << "pub 3" << std::endl;

                        cmd_pub->publish(cmd_vel); // 명령어를 발행합니다.
                    }
                }
                else if (lidar_flag == false && goal_arrived_flag == false)
                {
                    x_err(0) = target_pose_[0] - curr_pose_[0];
                    x_err(1) = target_pose_[1] - curr_pose_[1];
                    float distance_error = std::sqrt(float(x_err(0)) * float(x_err(0)) + float(x_err(1)) * float(x_err(1))); // 목표까지의 거리를 계산
                    Eigen::VectorXd robot_x_err = Eigen::VectorXd::Zero(3);                                                  // 로봇 프레임 기준 pose err
                    Eigen::Matrix3d rotation_matrix = createZRotationMatrix(robot_theta);                                    // 회전 행렬 생성
                    robot_x_err = rotation_matrix.transpose() * x_err;
                    float robot_target_angle = atan2(float(robot_x_err(1)), float(robot_x_err(0))); // 로봇 프레임 기준 타겟까지 각도
                    // std::cout << "[distance_error: " << distance_error<< ", rotation_error: " << robot_target_angle << "]" << std::endl;

                    if (robot_x_err(0) >= 0)
                    {
                        if (distance_error > position_error_tolerance)
                        {
                            // std::cout << "err 7" << std::endl;
                            v = checkLinearLimitVelocity(linear_vel_gain * distance_error + 0.04, max_linear_vel);    // 속도를 거리 오차에 비례하게 조절
                            v_yaw = checkLinearLimitVelocity(angular_vel_gain * robot_target_angle, max_angular_vel); // 목표 방향으로의 회전을 조금 적용
                        }
                        else
                        {
                            // std::cout << "err 8" << std::endl;

                            v = 0.0;                                                                     // 위치에 도달했으므로 전진 속도를 0으로 설정
                            v_yaw = checkLinearLimitVelocity(1.0 * robot_target_angle, max_angular_vel); // 목표 yaw를 정확히 맞추기 위해 회전 속도를 조절

                            if (std::abs(robot_target_angle) < 0.05)
                            {
                                v_yaw = 0.0;
                                RCLCPP_INFO(this->get_logger(), "Yaw aligned. Arrived at the target position and orientation.");
                            }
                        }
                    }
                    // else
                    // {
                    //     if (robot_target_angle > 0)
                    //     {
                    //         std::cout << "err 9" << std::endl;

                    //         v = checkLinearLimitVelocity(-0.1 * linear_vel_gain * distance_error, max_linear_vel);             // 속도를 거리 오차에 비례하게 조절
                    //         v_yaw = checkLinearLimitVelocity(angular_vel_gain * (robot_target_angle - M_PI), max_angular_vel); // 목표 방향으로의 회전을 조금 적용
                    //     }
                    //     else if (robot_target_angle < 0)
                    //     {
                    //         std::cout << "err 10" << std::endl;

                    //         v = checkLinearLimitVelocity(-0.1 * linear_vel_gain * distance_error, max_linear_vel);             // 속도를 거리 오차에 비례하게 조절
                    //         v_yaw = checkLinearLimitVelocity(angular_vel_gain * (robot_target_angle + M_PI), max_angular_vel); // 목표 방향으로의 회전을 조금 적용
                    //     }
                    //     else
                    //     {
                    //         std::cout << "err 11" << std::endl;

                    //         v = checkLinearLimitVelocity(-0.1 * linear_vel_gain * distance_error, max_linear_vel); // 속도를 거리 오차에 비례하게 조절
                    //         v_yaw = 0.0;
                    //     }
                    // }
                    // std::cout << "[v: " << v << ", v_yaw: " << v_yaw << "]" << std::endl;

                    auto cmd_vel = geometry_msgs::msg::Twist();
                    cmd_vel.linear.x = v;
                    cmd_vel.angular.z = v_yaw;
                    // std::cout << "pub 4" << std::endl;

                    cmd_pub->publish(cmd_vel); // 명령어를 발행합니다.
                }
            }
            // 도착함을 publish
            else if (goal_arrived_flag == true)
            {
                std_msgs::msg::String goal_status_msg;
                goal_status_msg.data = std::to_string(goal_arrived_flag) + "," + std::to_string(way_point);
                goal_status_publisher->publish(goal_status_msg);

                // 목표에 도달했으므로 로봇을 멈춤
                geometry_msgs::msg::Twist stop_msg;
                stop_msg.linear.x = 0.0;
                stop_msg.angular.z = 0.0;
                cmd_pub->publish(stop_msg);

                RCLCPP_INFO(this->get_logger(), "Goal reached, stopping.");
            }
            if (last_arrived == true)
            {
                goal_arrived_flag = true;
                geometry_msgs::msg::Twist stop_msg;
                stop_msg.linear.x = 0.0;
                stop_msg.angular.z = 0.0;
                // std::cout << "pub 5" << std::endl;
                target_flag = false;

                cmd_pub->publish(stop_msg);
            }
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // ROS2 노드를 초기화합니다.

    // // 로봇 이름을 변수로 설정합니다.
    // std::string robot_name = "robot1"; // 원하는 로봇 이름으로 변경

    auto node = std::make_shared<RobotController>(); // RobotController 노드를 생성합니다.
    rclcpp::spin(node);                              // 노드를 스핀하여 콜백 함수들이 실행되도록 합니다.
    rclcpp::shutdown();                              // ROS2 노드를 종료합니다.
    return 0;
}
