#include <iostream>
#include <vector>
#include <cmath>
#include "std_msgs/msg/int16_multi_array.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Dense>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "std_msgs/msg/int8.hpp"
#include "spline.h" // Include tk::spline header file
using namespace std;
#define DEG2RAD(a) ((a) * (M_PI / 180.0f))

// Function to split a string by a delimiter
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

// Function to normalize an angle to the range [-π, π]
void degree_repair(double &angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
}

// Function to normalize an angle difference to the range [-π, π]
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Definition of PID controller class
class PIDController
{
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

    // Function to compute the control signal using PID control
    double compute(double setpoint, double measured_value, double dt)
    {
        double error = setpoint - measured_value;
        error = normalizeAngle(error); // Normalize angle difference
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        prev_error_ = error;
        return output;
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double prev_error_;
    double integral_;
};

// Definition of Controller class
class Controller : public rclcpp::Node
{
public:
    Controller() : Node("controller"), target_index_(0), docking_flag(false), current_target_index(0)
    {
        // Subscribe to "target_position" topic
        target_subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "target_position", 10, std::bind(&Controller::targetPositionCallback, this, std::placeholders::_1));

        // Subscribe to "odom" topic
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "final_pose", 10, std::bind(&Controller::odom_callback, this, std::placeholders::_1));

        // Create publisher for "cmd_vel" topic
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Subscribe to "/scan" topic with QoS settings
        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "converted_scan", rclcpp::SensorDataQoS(), std::bind(&Controller::laser_callback, this, std::placeholders::_1));

        // Subscribe to "docking_position" topic
        dockingPostion_subscription = this->create_subscription<std_msgs::msg::String>(
            "docking_position", 10, std::bind(&Controller::dockingPositionCallback, this, std::placeholders::_1));

        // Subscribe to "backward_vel" topic
        backward_subscription = this->create_subscription<std_msgs::msg::Int16MultiArray>(
                "backward_vel", 10, std::bind(&Controller::backwardCallback, this, std::placeholders::_1));

        // Set timer to call publish_command function every 0.1 seconds
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Controller::publish_command, this));

        // Initialize current and target positions
        curr_pose_ = Eigen::Vector2d::Zero();
        target_pose_ = Eigen::Vector2d::Zero();

        // Initialize PID controllers (with appropriate PID parameters)
        pid_controller_ = std::make_shared<PIDController>(0.45, 0.0, 0.01); // kp, ki, kd values
        docking_pid_controller_ = std::make_shared<PIDController>(0.15, 0.0, 0.01); // kp, ki, kd values

    }

private:
    Eigen::Vector2d target_pose_;    // Target position
    Eigen::Vector2d curr_pose_;      // Current position
    double robot_theta = 0.0;        // Robot's current orientation
    double linear_velocity_x = 0.0;  // Robot's linear velocity
    double angular_velocity_z = 0.0; // Robot's angular velocity
    float REAL_X = 2.57;
    float REAL_Y = 1.94; // m
    int batch_size = 4;
    float fixelConst_x = REAL_X / (640 / batch_size); // 0.018 m
    float fixelConst_y = REAL_Y / (480 / batch_size); // m
    int target_index_;                                // Current target index
    std::vector<double> x_spline_;                    // x coordinates of the path
    std::vector<double> y_spline_;                    // y coordinates of the path
    std::vector<double> docking_x_spline_;            // x coordinates of the docking path
    std::vector<double> docking_y_spline_;            // y coordinates of the docking path
    bool wayCallback_flag = true;
    int target_yaw = 0.0;
    bool arrived_flag = true;
    bool docking_flag = false;
    bool web_move_flag = false;
    bool backward_flag = false;
    const double max_linear_velocity_ = 0.1;
    const double max_angular_velocity = 0.5;
    const double dt_ = 0.1;
    float desried_backDist = 0.5;
    int backwardTime = 0;
    std::vector<double> backward_init = {0.0,  0.0};
    // lidar parameter
    bool lidar_flag = false;
    int lidar_N = 0;
    bool firstinit = false;
    float *lidar_x;
    float *lidar_y;
    float *ranges;
    float min_range = 0.1;
    int min_index = 0;
    float too_close_distance = 0.5; // Distance to avoid collision

    std::vector<Eigen::Vector2f> path_points_;
    int current_target_index;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_; // Publisher
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dockingPostion_subscription;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr backward_subscription;

    rclcpp::TimerBase::SharedPtr timer_; // Timer

    std::shared_ptr<PIDController> pid_controller_; // PID controller
    std::shared_ptr<PIDController> docking_pid_controller_; // PID controller


    // Callback function for "target_position" topic
    void targetPositionCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
        std::vector<Eigen::Vector2d> target_poses;

        // Check if the message length is 10
        if (msg->data.size() == 10)
        {
            // Extract the first 8 values (coordinates) from the data array
            for (size_t i = 0; i < 8; i += 2)
            {
                // Convert coordinates from planning frame to world frame
                target_poses.emplace_back(msg->data[i + 1] * fixelConst_x, msg->data[i] * fixelConst_y);
            } // Convert pixel values to actual odometry values

            // Extract the last yaw and arrived_flag
            target_yaw = msg->data[8];
            arrived_flag = static_cast<bool>(msg->data[9]);

            // Process the path points after receiving new target positions
            if (wayCallback_flag && !arrived_flag)
            {
                process_points_bezier(target_poses);
                wayCallback_flag = false;
            }
            web_move_flag = true;

        }
        else if (msg->data.size() == 8)
        {
            // Extract the first 6 values (coordinates) from the data array
            for (size_t i = 0; i < 6; i += 2)
            {
                // Convert coordinates from planning frame to world frame
                target_poses.emplace_back(msg->data[i + 1] * fixelConst_x, msg->data[i] * fixelConst_y);
            } // Convert pixel values to actual odometry values

            // Extract the last yaw and arrived_flag
            target_yaw = msg->data[6];
            arrived_flag = static_cast<bool>(msg->data[7]);

            // Process the path points after receiving new target positions
            if (wayCallback_flag && !arrived_flag)
            {
                process_points_bezier(target_poses);
                wayCallback_flag = false;
            }
            web_move_flag = true;

        }
        else if (msg->data.size() == 6)
        {
            // Extract the first 4 values (coordinates) from the data array
            for (size_t i = 0; i < 4; i += 2)
            {
                target_poses.emplace_back(msg->data[i + 1] * fixelConst_x, msg->data[i] * fixelConst_y);
            } // Convert pixel values to actual odometry values

            // Extract the last yaw and arrived_flag
            target_yaw = msg->data[4];
            arrived_flag = static_cast<bool>(msg->data[5]);

            // Process the path points after receiving new target positions
            if (wayCallback_flag && !arrived_flag)
            {
                process_points_bezier(target_poses);
                wayCallback_flag = false;
            }
            web_move_flag = true;

        }
        else if (msg->data.size() == 2)
        {
            // If the message length is 2, extract only the target_yaw and arrived_flag
            target_yaw = msg->data[0];
            arrived_flag = static_cast<bool>(msg->data[1]);
            web_move_flag = false;

        }
        else
        {
            std::cout << "Invalid message size: " << msg->data.size() << std::endl;
        }
    }

    // Callback function for "docking_position" topic
    void dockingPositionCallback(const std_msgs::msg::String::ConstPtr &msg)
    {
        double dockingPose_[3] = {0, 0, 0};
        std::vector<std::string> dockingPose_data = split(msg->data, ',');
        if (dockingPose_data.size() < 3)
            return;

        dockingPose_[0] = std::stod(dockingPose_data[0]) * fixelConst_x;
        dockingPose_[1] = std::stod(dockingPose_data[1]) * fixelConst_y;
        dockingPose_[2] = std::stod(dockingPose_data[2]);

        // Convert coordinates from planning frame to world frame
        double temp_ = dockingPose_[0];
        dockingPose_[0] = dockingPose_[1];
        dockingPose_[1] = temp_;

        // Create P1, P2, P3 points
        Eigen::Vector2f P1(curr_pose_[0], curr_pose_[1]);
        Eigen::Vector2f P3(dockingPose_[0], dockingPose_[1]);
        float docking_yaw = dockingPose_[2];
        std::cout << "curr: " << curr_pose_[0] << " " << curr_pose_[1] << std::endl;
        std::cout << "dock: " << dockingPose_[0] << " " << dockingPose_[1] << std::endl;

        float distance = 0.1; // Set desired distance in meters

        // Call the function to generate the docking path
        dockingProcess_points_bezier(P1, P3, docking_yaw, distance);
        docking_flag = true;
        std::cout << "docking Callback DONE!!" << std::endl;
    }

    // Function to adjust P3 based on yaw and distance
    Eigen::Vector2f adjustP3ForYaw(const Eigen::Vector2f &P3, float yaw, float distance)
    {   
        float deltaY = 0.0;
        float deltaX = 0.0;
        double target_yaw = check_target_yaw(yaw);
        if(yaw==1)
        {  
            deltaX = std::cos(target_yaw) * distance;
            return Eigen::Vector2f(P3.x() - abs(deltaX), P3.y() );
        }
        else if (yaw ==-1)
        {
            deltaX = std::cos(target_yaw) * distance;
            return Eigen::Vector2f(P3.x()+ abs(deltaX), P3.y() );
        }
        else if (yaw==2)
        {
            deltaY = std::sin(target_yaw) * distance;
            return Eigen::Vector2f(P3.x(), P3.y() + abs(deltaY));
        }
        else if (yaw==-2)
        {
            deltaY = std::sin(target_yaw) * distance;
            return Eigen::Vector2f(P3.x(), P3.y() - abs(deltaY));
        }
        else std::cout << "adjust can't working!!" << std::endl;
        
    }

    // Function to calculate Bezier curve
    Eigen::Vector2d bezierCurve(double t, const std::vector<Eigen::Vector2d> &control_points)
    {
        int n = control_points.size() - 1;
        Eigen::Vector2d point(0.0, 0.0);
        for (int i = 0; i <= n; ++i)
        {
            double binomial_coeff = std::tgamma(n + 1) / (std::tgamma(i + 1) * std::tgamma(n - i + 1));
            double term = binomial_coeff * std::pow(t, i) * std::pow(1 - t, n - i);
            point += term * control_points[i];
        }
        return point;
    }

    // Function to process points and generate Bezier path
    void process_points_bezier(const std::vector<Eigen::Vector2d> &points)
    {
        int n = points.size();
        std::vector<double> t(n);
        std::vector<double> x(n);
        std::vector<double> y(n);
        for (int i = 0; i < n; ++i)
        {
            t[i] = i;
            x[i] = points[i][0];
            y[i] = points[i][1];
        }

        // Calculate the total distance of the path
        double total_distance = 0;
        for (int i = 0; i < n - 1; ++i)
        {
            double dx = x[i + 1] - x[i];
            double dy = y[i + 1] - y[i];
            total_distance += std::sqrt(dx * dx + dy * dy);
        }

        // Calculate the number of samples and sampling interval
        double v_max = 0.1; // Maximum linear velocity (m/s)
        double dt = 0.1;    // Sampling time interval (s)
        int num_samples = static_cast<int>(total_distance / (v_max * dt));

        // Set minimum number of samples
        int min_samples = 10;
        if (num_samples < min_samples)
        {
            num_samples = min_samples; // Use minimum value if less than the minimum number of samples
        }

        std::vector<double> t_samples = linspace(0, 1, num_samples); // Sample from 0 to 1

        // Calculate the sampled points
        x_spline_.clear();
        y_spline_.clear();
        for (const auto &t : t_samples)
        {
            Eigen::Vector2d point = bezierCurve(t, points);
            x_spline_.push_back(point.x());
            y_spline_.push_back(point.y());
        }
        target_index_ = 1;
    }

    // Function to process points and generate docking path using Bezier curve
    void dockingProcess_points_bezier(const Eigen::Vector2f &P1, const Eigen::Vector2f &P3, float docking_yaw, float distance)
    {
        // Calculate P2 based on P3 and yaw
        Eigen::Vector2f P2_float = adjustP3ForYaw(P3, docking_yaw, distance);
        Eigen::Vector2d P1_d = P1.cast<double>();
        Eigen::Vector2d P2_d = P2_float.cast<double>();
        Eigen::Vector2d P3_d = P3.cast<double>();

        // Control points for Bezier curve from P1 to P2
        std::vector<Eigen::Vector2d> control_points = {P1_d, P2_d};

        // Calculate total distance for Bezier segment
        double bezier_distance = (P1_d - P2_d).norm();

        // Sampling parameters
        double v_max = 0.2; // Maximum linear velocity (m/s)
        double dt = 0.5;    // Sampling time interval (s)
        int num_samples = static_cast<int>(bezier_distance / (v_max * dt));
        int min_samples = 2;
        if (num_samples < min_samples)
        {
            num_samples = min_samples;
        }

        // Sample points on Bezier curve
        std::vector<double> t_samples = linspace(0, 1, num_samples);
        docking_x_spline_.clear();
        docking_y_spline_.clear();

        for (const auto &t : t_samples)
        {
            Eigen::Vector2d point = bezierCurve(t, control_points);
            docking_x_spline_.push_back(point.x());
            docking_y_spline_.push_back(point.y());
        }

        // Add linear segment from P2 to P3
        int linear_samples = 2; // Ensure enough points for the linear segment
        std::vector<double> t_linear_samples = linspace(0, 1, linear_samples);
        for (const auto &t : t_linear_samples)
        {
            if(0!=t)
            {
            double x = (1 - t) * P2_d.x() + t * P3_d.x();
            double y = (1 - t) * P2_d.y() + t * P3_d.y();
            docking_x_spline_.push_back(x);
            docking_y_spline_.push_back(y);
            }
        }

        // Set target_index_ to 1 (example)
        target_index_ = 1;
    }

    // Callback function to process lidar data
    void laser_callback(const sensor_msgs::msg::LaserScan::ConstPtr &msg)
    {
        lidar_flag = false;
        lidar_N = (int)((msg->angle_max - msg->angle_min) / msg->angle_increment) + 1;

        // First initialization - locate memory for table
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
            if (!std::isnan(ranges[i]) && !std::isinf(ranges[i]) && ranges[i] > min_range)
            {
                if (ranges[i] < min_value)
                {
                    // Store the closest index
                    min_index = i;
                    min_value = ranges[i];
                }

                float xl = cos(msg->angle_min + i * msg->angle_increment) * ranges[i];
                float yl = sin(msg->angle_min + i * msg->angle_increment) * ranges[i];

                // Calculate obstacle global position
                lidar_x[i] = curr_pose_[0] + xl * cos(robot_theta) - yl * sin(robot_theta);
                lidar_y[i] = curr_pose_[1] + xl * sin(robot_theta) + yl * cos(robot_theta);
            }
        }
        lidar_flag = (min_value != 9999); // Set lidar_flag to true if there is any valid reading
    }

    // Callback function to update robot's position and orientation
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!msg)
        {
            RCLCPP_ERROR(this->get_logger(), "Received null odometry message");
            return; // Check if msg is valid
        }
        curr_pose_[0] = msg->pose.pose.position.y; 
        curr_pose_[1] = msg->pose.pose.position.x;
        robot_theta = 2.0 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        linear_velocity_x = msg->twist.twist.linear.x;   // Linear velocity x
        angular_velocity_z = msg->twist.twist.angular.z; // Angular velocity z
        degree_repair(robot_theta);                      // Normalize the angle to the range [-π, π]
    }

    // Callback function for "backward_vel" topic
    void backwardCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
        backwardTime = msg->data[0]; // time
        // Convert to world frame
        backward_init[0] = double(msg->data[2])*fixelConst_x; // init x
        backward_init[1] = double(msg->data[1])*fixelConst_y; // init y

        backward_flag = true;
        // Create P1, P2, P3 points
        Eigen::Vector2f P1(curr_pose_[0], curr_pose_[1]);
        Eigen::Vector2f P3(backward_init[0], backward_init[1]);
        float docking_yaw = -1;
        float distance = 0.05; // Set desired distance in meters

        // Call the function to generate the docking path
        dockingProcess_points_bezier(P1, P3, docking_yaw, distance);
    }
    // Function to generate a linear space between start and end
    std::vector<double> linspace(double start, double end, int num)
    {
        std::vector<double> linspaced;
        if (num == 0)
        {
            return linspaced;
        }
        if (num == 1)
        {
            linspaced.push_back(start);
            return linspaced;
        }
        double delta = (end - start) / (num - 1);
        for (int i = 0; i < num - 1; ++i)
        {
            linspaced.push_back(start + delta * i);
        }
        linspaced.push_back(end);
        return linspaced;
    }

    // Function to check and return the target yaw
    double check_target_yaw(int target_yaw)
    {
        double desire_yaw = 0.0;
        if (target_yaw == 1)
        {
            desire_yaw = 0.0;
        }
        else if (target_yaw == -1)
        {
            desire_yaw = M_PI;
        }
        else if (target_yaw == 2)
        {
            desire_yaw = -M_PI / 2;
        }
        else if (target_yaw == -2)
        {
            desire_yaw = M_PI / 2;
        }
        return desire_yaw;
    }

    // Function to turn the robot to the desired yaw
    void turn(double desire_yaw)
    {
        // Calculate the angle difference between current and target yaw
        double angle_difference = desire_yaw - robot_theta;
        if(desire_yaw == M_PI && angle_difference > M_PI)
        {
            desire_yaw = -M_PI;
        }
        angle_difference = desire_yaw - robot_theta;

        // Normalize the angle difference to the range [-π, π]
        while (angle_difference > M_PI)
            angle_difference -= 2 * M_PI;
        while (angle_difference < -M_PI)
            angle_difference += 2 * M_PI;


        // Rotate the robot initially before moving
        geometry_msgs::msg::Twist cmd_msg;
        if (std::abs(angle_difference) > 0.035) 
        {
            // Set linear velocity to 0 and rotate in place
            cmd_msg.linear.x = 0.0;
            // Set angular velocity based on angle difference (simple example)
            cmd_msg.angular.z = angle_difference > 0 ? 0.15 : -0.15; // Adjust angular velocity as needed
        }
        else
        {
            // Set linear and angular velocities to 0 if facing the target direction
            cmd_msg.linear.x = 0.0; // Adjust as needed
            cmd_msg.angular.z = 0.0;
        }
        cmd_vel_publisher_->publish(cmd_msg);
    }

    // Function to follow the target path
    void followtarget()
    {   
        wayCallback_flag = true; 
        // Stop if the target index exceeds the path size
        if (target_index_ >= static_cast<int>(x_spline_.size()))
        {
            wayCallback_flag = true;
            return;
        }

        // Calculate the current and next target positions
        Eigen::Vector2d current_position(curr_pose_[0], curr_pose_[1]);
        Eigen::Vector2d target_position(x_spline_[target_index_], y_spline_[target_index_]);
        Eigen::Vector2d last_target_position(x_spline_[target_index_ - 1], y_spline_[target_index_ - 1]);

        // Calculate the distance to the target position
        double distance = (target_position - current_position).norm();
        double theta_target = std::atan2(target_position[1] - last_target_position[1], target_position[0] - last_target_position[0]); // Normalize angle difference to the range [-π, π]
        double angle_diff = theta_target - robot_theta;
        double max_speed = 0.08; // Maximum linear velocity
        double min_speed = 0.0;  // Minimum linear velocity
        double max_angular_speed = 0.5;
        double omega = pid_controller_->compute(theta_target, robot_theta, dt_);
        omega = std::min(std::max(omega, -max_angular_speed), max_angular_speed);
        double speed_factor = std::max(1.0 - std::abs(omega) / max_angular_speed, 0.0); // Speed factor approaches 1 as angle_diff approaches 0
        double v = min_speed + (max_speed - min_speed) * speed_factor;                  // Adjust v based on speed factor

        // Create and publish the command message
        auto cmd_vel_msg = geometry_msgs::msg::Twist();
        cmd_vel_msg.linear.x = v;
        cmd_vel_msg.angular.z = omega;
        cmd_vel_publisher_->publish(cmd_vel_msg);
        target_index_++;
    }

    // Function to perform docking maneuver
    void dockingMoveFunction()
    {
        // Calculate the current and next target positions
        Eigen::Vector2d current_position(curr_pose_[0], curr_pose_[1]);
        Eigen::Vector2d target_position(docking_x_spline_[target_index_], docking_y_spline_[target_index_]);
        Eigen::Vector2d last_target_position(docking_x_spline_[target_index_ - 1], docking_y_spline_[target_index_ - 1]);

        // Calculate the distance to the target position
        double distance = (target_position - current_position).norm();

        // Move to the next target if the current target is close enough
        if (distance < 0.01) // Move to the next target if within 1cm
        {
            target_index_++;
            if (target_index_ >= static_cast<int>(docking_x_spline_.size()))
            {
                docking_flag = false; 
                arrived_flag = false;
                auto cmd_vel_msg = geometry_msgs::msg::Twist();
                cmd_vel_msg.linear.x = 0.0; // Adjust as needed
                cmd_vel_msg.angular.z = 0.0;
                cmd_vel_publisher_->publish(cmd_vel_msg);

                RCLCPP_INFO(this->get_logger(), "Now robot is TABLE");
                std::cout << "curr GRID: " << curr_pose_[0] / fixelConst_y << " " << curr_pose_[1] / fixelConst_x << std::endl;

                return;
            }
        }

        
        // Normalize the angle difference to the range [-π, π]
        double theta_target = std::atan2(target_position[1] - current_position[1], target_position[0] - current_position[0]);
        double angle_diff = theta_target - robot_theta;
        while (angle_diff > M_PI)
            angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI)
            angle_diff += 2.0 * M_PI;

        double max_speed = 0.03; // Maximum linear velocity
        double min_speed = 0.0;  // Minimum linear velocity
        double max_angular_speed = 0.15;
        double omega = docking_pid_controller_->compute(theta_target, robot_theta, dt_);
        omega = std::min(std::max(omega, -0.15), 0.15);
        double speed_factor = std::max(1.0 - std::abs(omega) / max_angular_speed, 0.0); // Speed factor approaches 1 as angle_diff approaches 0
        double distance_factor = std::min(distance / hypot(target_position[1] - current_position[1], target_position[0] - current_position[0]), 1.0); // Distance factor approaches 1 as distance approaches 0
        double v = min_speed + (max_speed - min_speed) * speed_factor * distance_factor;                  // Adjust v based on speed and distance factors

        // Create and publish the command message
        auto cmd_vel_msg = geometry_msgs::msg::Twist();

        double min_linear_vel = 0.02;
        if (abs(angle_diff) > 0.035) 
        {
            cmd_vel_msg.linear.x = 0.0;
            double min_angular_vel = 0.07;
            if(abs(omega) > min_angular_vel) // If the calculated angular velocity is too small, use a minimum angular velocity
            {
                cmd_vel_msg.angular.z = omega;
            }
            else
            {
                if(omega > 0.0)
                {
                    cmd_vel_msg.angular.z = min_angular_vel;
                }
                else if(omega < 0.0)
                {
                    cmd_vel_msg.angular.z = -min_angular_vel;
                }
                else
                {
                    cmd_vel_msg.angular.z = 0.0;
                }
            }
        }
        else if (distance > 0.01)
        {
            if(v < min_linear_vel) // If the calculated linear velocity is too small, use a minimum linear velocity
            {
                v = min_linear_vel;
            }
            cmd_vel_msg.linear.x = v;   
            cmd_vel_msg.angular.z = 0.0;
        }
        else{
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = 0.0;
        }

        cmd_vel_publisher_->publish(cmd_vel_msg);
    }

    // Function to perform backward maneuver
    void backwardMoveFunction()
    {
        // Calculate the current and next target positions
        Eigen::Vector2d current_position(curr_pose_[0], curr_pose_[1]);
        Eigen::Vector2d target_position(docking_x_spline_[target_index_], docking_y_spline_[target_index_]);
        Eigen::Vector2d last_target_position(docking_x_spline_[target_index_ - 1], docking_y_spline_[target_index_ - 1]);

        // Calculate the distance to the target position
        double distance = (target_position - current_position).norm();

        // Move to the next target if the current target is close enough
        if (distance < 0.01) // Move to the next target if within 1cm
        {
            target_index_++;
            RCLCPP_INFO(this->get_logger(), "backward distance < 0.01");
            std::cout << "target_index_: " << target_index_ << std::endl; 


            if (target_index_ >= static_cast<int>(docking_x_spline_.size()))
            {
                docking_flag = false;
                arrived_flag = false;
                auto cmd_vel_msg = geometry_msgs::msg::Twist();
                cmd_vel_msg.linear.x = 0.0; // Adjust as needed
                cmd_vel_msg.angular.z = 0.0;
                cmd_vel_publisher_->publish(cmd_vel_msg);

                RCLCPP_INFO(this->get_logger(), "Now robot is TABLE@@");
                std::cout << "curr GRID: " << curr_pose_[0] / fixelConst_y << " " << curr_pose_[1] / fixelConst_x << std::endl;

                return;
            }
        }

        
        // Normalize the angle difference to the range [-π, π]
        double theta_target = std::atan2(target_position[1] - current_position[1], target_position[0] - current_position[0]);
        if(theta_target > DEG2RAD(90))
        {
            theta_target = theta_target - M_PI;
        }
        else if(theta_target < -1 * DEG2RAD(90))
        {
            theta_target = theta_target + M_PI;

        }
        else
        {
            theta_target = 0.0;
        }

        double angle_diff = theta_target - robot_theta;
        while (angle_diff > M_PI)
            angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI)
            angle_diff += 2.0 * M_PI;

        double max_speed = 0.03; // Maximum linear velocity
        double min_speed = 0.0;  // Minimum linear velocity
        double max_angular_speed = 0.15;
        double omega = docking_pid_controller_->compute(theta_target, robot_theta, dt_);
        omega = std::min(std::max(omega, -0.15), 0.15);
        double speed_factor = std::max(1.0 - std::abs(omega) / max_angular_speed, 0.0); // Speed factor approaches 1 as angle_diff approaches 0
        double distance_factor = std::min(distance / hypot(target_position[1] - current_position[1], target_position[0] - current_position[0]), 1.0); // Distance factor approaches 1 as distance approaches 0
        double v = -1 * (max_speed - min_speed) * speed_factor * distance_factor;                  // Adjust v based on speed and distance factors

        // Create and publish the command message
        auto cmd_vel_msg = geometry_msgs::msg::Twist();

        double min_linear_vel = -0.02;
        if (abs(angle_diff) > 0.035) //0.02 is within 2 degrees to match yaw
        {
            cmd_vel_msg.linear.x = 0.0;
            double min_angular_vel = 0.07;
            if(abs(omega) > min_angular_vel) // If the calculated angular velocity is too small, use a minimum angular velocity
            {
                cmd_vel_msg.angular.z = omega;
            }
            else
            {
                if(omega > 0.0)
                {
                    cmd_vel_msg.angular.z = min_angular_vel;
                }
                else if(omega < 0.0)
                {
                    cmd_vel_msg.angular.z = -min_angular_vel;
                }
                else
                {
                    cmd_vel_msg.angular.z = 0.0;
                }
            }
        }
        else if (distance > 0.01)
        {
            if(v > min_linear_vel) // If the calculated linear velocity is too small, use a minimum linear velocity
            {
                v = min_linear_vel;
            }
            cmd_vel_msg.linear.x = v;   
            cmd_vel_msg.angular.z = 0.0;
        }
        else{
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = 0.0;
        }
        cmd_vel_publisher_->publish(cmd_vel_msg);
    }

    // Function to check for obstacles using lidar
    void lidar_check()
    {
        float obstacle_distance = hypot(curr_pose_[1] - lidar_y[min_index], curr_pose_[0] - lidar_x[min_index]);
        if (obstacle_distance < too_close_distance)
        {
            auto cmd_vel = geometry_msgs::msg::Twist();
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_publisher_->publish(cmd_vel); // Publish the stop command
        }
        else
        {
            followtarget();
        }
    }

    // Function to calculate and publish linear and angular velocities for each segment
    void publish_command()
    {
        if(backward_flag)
        {
            auto cmd_vel = geometry_msgs::msg::Twist();
            if (backwardTime > 0)
            {
                RCLCPP_INFO(this->get_logger(), "UnDOCKING back ward,,, ");
                RCLCPP_INFO(this->get_logger(), "backwardTime", backwardTime);
                std::cout << "backwardTime"<< backwardTime << std::endl;


                cmd_vel.linear.x = -0.1;
                cmd_vel.angular.z = 0.0;
                cmd_vel_publisher_->publish(cmd_vel); // Publish the command
                
                backwardTime--; 
                if(backwardTime == 0)
                {
                    backward_flag = false;
                    wayCallback_flag = true;
                    // arrived_flag =false;
                }
            }
            else if (hypot(backward_init[0] - curr_pose_[0],backward_init[1] -  curr_pose_[1]) > 0.02) 
            {
                
                backwardMoveFunction();
                RCLCPP_INFO(this->get_logger(), "backward to INIT POSE,,, ");
            }
            else
            { 
                RCLCPP_INFO(this->get_logger(), "Now robot INIT POSE !!");
                backward_flag = false;
                target_yaw = 1;
                arrived_flag = 1;
                docking_x_spline_.clear();
                docking_y_spline_.clear();
                docking_flag = false;
                web_move_flag = false;
            }
  
        }
        else{
            // If arrived_flag is true, process arrival
            if (docking_flag == true)
            {
                dockingMoveFunction();
            }
            else if (arrived_flag)
            {
                double desire_yaw = check_target_yaw(target_yaw);
                turn(desire_yaw);
            }
            else if(web_move_flag == true)
            {

                if (lidar_flag == true)
                {
                    lidar_check();
                }
                else
                {
                    followtarget();
                }
            }
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr target_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
