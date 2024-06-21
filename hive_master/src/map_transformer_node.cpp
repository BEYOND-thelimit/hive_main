#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

// Definition: Occupancy state values
constexpr int OBSTACLE = 100; // Value considered as an obstacle
constexpr int UNOCCUPIED = 0; // Value considered as unoccupied space

// The makeNewMap function removes a specific color from the map and performs circular expansion for other colors.
Eigen::MatrixXi makeNewMap(const Eigen::MatrixXi &map, int robot_color, int distance)
{
    int obstacle_value = OBSTACLE; // Obstacle value
    Eigen::MatrixXi new_map = map; // Create a copy of the input map to avoid modifying the original data.
    const int rows = map.rows(); // Get the number of rows in the map.
    const int cols = map.cols(); // Get the number of columns in the map.
    const int distance_squared = distance * distance; // Pre-calculate the square of the distance for circular expansion.

    std::vector<std::pair<int, int>> robot_positions; // Vector to store positions where the robot color is found.
    // Iterate through all cells in the map.
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            // If the current cell matches the robot color, store its position in the robot_positions vector.
            if (map(i, j) == robot_color)
            {
                std::cout << "robot in here!!" << std::endl;
                robot_positions.push_back(std::make_pair(i, j));
            }
            // If the current cell is not the robot color, and not empty (0) or an existing obstacle (1), perform the expansion logic.
            else if (map(i, j) != 0 && map(i, j) != 1)
            {
                // Attempt to expand around the current cell up to the specified distance.
                for (int x = std::max(0, i - distance); x <= std::min(i + distance, rows - 1); ++x)
                {
                    for (int y = std::max(0, j - distance); y <= std::min(j + distance, cols - 1); ++y)
                    {
                        int dx = x - i; // x distance from the current cell
                        int dy = y - j; // y distance from the current cell
                        // Check if within the specified distance using the squared Euclidean distance.
                        if (dx * dx + dy * dy <= distance_squared)
                        {
                            new_map(x, y) = obstacle_value; // If within the distance, set the cell as an obstacle.
                        }
                    }
                }
            }
            else if (map(i, j) == 1)
                new_map(i, j) = obstacle_value; // Set existing obstacles to the obstacle value.
        }
    }

    // Iterate through all positions where the robot color was found and remove the color.
    for (const auto &pos : robot_positions)
    {
        new_map(pos.first, pos.second) = 0; // Set the robot color positions to empty space.
    }

    return new_map; // Return the modified map.
}

// Function to reduce the map
Eigen::MatrixXi batchProcessMap(const Eigen::MatrixXi &map, int batch)
{
    const int rows = map.rows(); // Number of rows in the original map
    const int cols = map.cols(); // Number of columns in the original map
    int new_rows = rows / batch; // Number of rows in the reduced map
    int new_cols = cols / batch; // Number of columns in the reduced map

    Eigen::MatrixXi reduced_map = Eigen::MatrixXi::Zero(new_rows, new_cols); // Initialize the reduced map

    // Reduce the map by the batch size
    for (int j = 0; j < new_rows; ++j)
    {
        for (int i = 0; i < new_cols; ++i)
        {
            int max_value = UNOCCUPIED; // Initial value is unoccupied space
            for (int bj = 0; bj < batch; ++bj)
            {
                for (int bi = 0; bi < batch; ++bi)
                {
                    max_value = std::max(max_value, map(j * batch + bj, i * batch + bi)); // Find the maximum value within the batch
                }
            }
            reduced_map(j, i) = (max_value > 0) ? OBSTACLE : UNOCCUPIED; // Set as obstacle or unoccupied space based on the maximum value
        }
    }

    return reduced_map; // Return the reduced map
}

class MapTransformerNode : public rclcpp::Node
{
public:
    MapTransformerNode() : Node("map_transformer_node")
    {
        // Declare and initialize parameters
        this->declare_parameter<std::string>("output_topic", "transformed_map");
        this->declare_parameter<int>("robot_color1", 2);
        this->declare_parameter<int>("robot_color2", 3);
        this->declare_parameter<int>("robot_color3", 4);
        this->declare_parameter<int>("expand_distance", 105); // Value increased by 5cm from the robot radius. 5cm = 11 pixels
        this->declare_parameter<int>("batch_size", 4); // Batch size

        int expand_distance, batch_size;
        int robot_color1, robot_color2, robot_color3;

        // Get parameter values
        this->get_parameter("robot_color1", robot_color1);
        this->get_parameter("robot_color2", robot_color2);
        this->get_parameter("robot_color3", robot_color3);
        this->get_parameter("expand_distance", expand_distance);
        this->get_parameter("batch_size", batch_size);

        // Create publishers
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        publisher1_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/robot1/occupancyGridmap", qos);
        publisher2_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/robot2/occupancyGridmap", qos);
        publisher3_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/robot3/occupancyGridmap", qos);

        // Create subscription and set callback
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", qos,
            [this, robot_color1, robot_color2, robot_color3, expand_distance, batch_size](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                std::cout << "Received map message" << std::endl; // Add debugging message
                processMap(msg, robot_color1, expand_distance, batch_size, publisher1_);
                processMap(msg, robot_color2, expand_distance, batch_size, publisher2_);
                processMap(msg, robot_color3, expand_distance, batch_size, publisher3_);
            });
    }

private:
    // Function to process OccupancyGrid message
    void processMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, int robot_color, int distance, int batch_size, rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr& publisher_) 
    {
        // Convert OccupancyGrid to Eigen matrix
        Eigen::MatrixXi map_matrix = occupancyGridToMatrix(msg);
        // Transform the map using robot color and distance information
        Eigen::MatrixXi transformed_map = makeNewMap(map_matrix, robot_color, distance);
        // Reduce the map by batch size
        Eigen::MatrixXi reduced_map = batchProcessMap(transformed_map, batch_size);
        // Convert the reduced map to OccupancyGrid message
        auto transformed_msg = matrixToOccupancyGrid(reduced_map, msg);
        // Publish the transformed map
        publisher_->publish(*transformed_msg);
    }

    // Function to convert OccupancyGrid message to Eigen matrix
    Eigen::MatrixXi occupancyGridToMatrix(const nav_msgs::msg::OccupancyGrid::SharedPtr &grid)
    {
        int rows = grid->info.height; // Number of rows in the map
        int cols = grid->info.width; // Number of columns in the map
        Eigen::MatrixXi matrix(rows, cols); // Initialize the matrix

        // Convert OccupancyGrid data to matrix
        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                matrix(i, j) = grid->data[i * cols + j];
            }
        }
        return matrix; // Return the converted matrix
    }

    // Function to convert Eigen matrix to OccupancyGrid message
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> matrixToOccupancyGrid(const Eigen::MatrixXi &matrix, const nav_msgs::msg::OccupancyGrid::SharedPtr &original_grid)
    {
        auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        *grid = *original_grid; // Copy metadata
        grid->data.clear();
        
        // Convert matrix data to OccupancyGrid data
        for (int i = 0; i < matrix.rows(); ++i)
        {
            for (int j = 0; j < matrix.cols(); ++j)
            {
                grid->data.push_back(matrix(i, j));
            }
        }
        // Update OccupancyGrid info
        grid->info.width = matrix.cols();
        grid->info.height = matrix.rows();
        return grid; // Return the converted OccupancyGrid message
    }

    // Member variable declarations
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_; // Map subscription
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher1_; // Robot 1 publisher
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher2_; // Robot 2 publisher
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher3_; // Robot 3 publisher
};

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // Initialize ROS2
    auto node = std::make_shared<MapTransformerNode>(); // Create node
    rclcpp::spin(node); // Spin node
    rclcpp::shutdown(); // Shutdown ROS2
    return 0;
}
