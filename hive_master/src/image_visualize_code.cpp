#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono> 
#include <algorithm>

// Function declaration: Expand all colors in the given map by a specified distance
Eigen::MatrixXi expandAllColors(const Eigen::MatrixXi& map, int distance) // distance is the number of pixels representing the robot's radius + safety distance (at least the robot's radius)
{ 
    Eigen::MatrixXi new_map = map; // Create a copy of the original map
    const int rows = map.rows(); // Number of rows in the map
    const int cols = map.cols(); // Number of columns in the map
    const int distance_squared = distance * distance; // Square of the distance

    // Iterate through all cells in the map
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            // Process cells that are not 0 or 1
            if (map(i, j) != 0 && map(i, j) != 1) {
                int cell_color = map(i, j); // Current cell color
                // Expand to all cells within the specified distance
                for (int x = std::max(0, i - distance); x <= std::min(i + distance, rows - 1); ++x) {
                    for (int y = std::max(0, j - distance); y <= std::min(j + distance, cols - 1); ++y) {
                        int dx = x - i; // Calculate x distance
                        int dy = y - j; // Calculate y distance
                        if (dx * dx + dy * dy <= distance_squared) { // Check if within the distance using squared Euclidean distance
                            new_map(x, y) = cell_color; // If within the distance, set the color to the current color
                        }
                    }
                }
            }
        }
    }

    return new_map; // Return the modified map
}

// makeNewMap function removes a specific color from the map and expands other colors in a circular manner.
Eigen::MatrixXi makeNewMap(const Eigen::MatrixXi& map, int robot_color, int distance) 
{
    Eigen::MatrixXi new_map = map; // Create a copy of the input map to avoid modifying the original data.
    const int rows = map.rows(); // Get the number of rows in the map.
    const int cols = map.cols(); // Get the number of columns in the map.
    const int distance_squared = distance * distance; // Pre-calculate the square of the distance for circular expansion.

    std::vector<std::pair<int, int>> robot_positions; // Vector to store the positions where the robot color is found.

    // Iterate through all cells in the map.
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            // If the current cell matches the robot color, store its position in the robot_positions vector.
            if (map(i, j) == robot_color) {
                robot_positions.push_back(std::make_pair(i, j));
            }
            // If the current cell is not the robot color, and not empty (0) or an existing obstacle (1), perform the expansion logic.
            else if (map(i, j) != 0 && map(i, j) != 1) {
                // Attempt to expand around the current cell up to the specified distance.
                for (int x = std::max(0, i - distance); x <= std::min(i + distance, rows - 1); ++x) {
                    for (int y = std::max(0, j - distance); y <= std::min(j + distance, cols - 1); ++y) {
                        int dx = x - i; // x distance from the current cell
                        int dy = y - j; // y distance from the current cell
                        // Check if within the specified distance using the squared Euclidean distance.
                        if (dx * dx + dy * dy <= distance_squared) {
                            new_map(x, y) = 1; // If within the distance, set the cell as an obstacle (1).
                        }
                    }
                }
            }
        }
    }

    // Iterate through all positions where the robot color was found and remove the color.
    for (const auto& pos : robot_positions) {
        new_map(pos.first, pos.second) = 0; // Set the robot color positions to empty space (0).
    }

    return new_map; // Return the modified map.
}

// Function declaration: Convert an Eigen matrix to an OpenCV image
cv::Mat matrixToImage(const Eigen::MatrixXi& matrix) 
{
    const int rows = matrix.rows(); // Number of rows
    const int cols = matrix.cols(); // Number of columns
    cv::Mat image(rows, cols, CV_8UC3); // Create an OpenCV image

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            cv::Vec3b color; // Set the color
            switch (matrix(r, c)) {
                case 0: color = cv::Vec3b(255, 255, 255); break; // White
                case 1: color = cv::Vec3b(0, 0, 0); break; // Black
                case 2: color = cv::Vec3b(0, 0, 255); break; // Red
                case 3: color = cv::Vec3b(0, 255, 0); break; // Green
                case 4: color = cv::Vec3b(255, 0, 0); break; // Blue
                default: color = cv::Vec3b(128, 128, 128); break; // Gray for others
            }
            image.at<cv::Vec3b>(cv::Point(c, r)) = color; // Assign the color to the image
        }
    }

    return image; // Return the created image
}

// Function declaration: Visualize an Eigen matrix
void visualizeMatrix(const Eigen::MatrixXi& matrix) 
{
    const int rows = matrix.rows();
    const int cols = matrix.cols();
    cv::Mat image(rows, cols, CV_8UC3);  // Create an OpenCV image

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            int value = matrix(r, c);
            cv::Vec3b color;
            switch (value) {
                case 1: color = cv::Vec3b(0, 0, 0); break;      // Black
                case 2: color = cv::Vec3b(0, 0, 255); break;    // Red (BGR order in OpenCV)
                case 3: color = cv::Vec3b(0, 255, 0); break;    // Green
                case 4: color = cv::Vec3b(255, 0, 0); break;    // Blue
                default: color = cv::Vec3b(255, 255, 255); break; // White
            }
            image.at<cv::Vec3b>(cv::Point(c, r)) = color;  // Assign the color to the corresponding pixel in the image
        }
    }

    // Display the image
    cv::imshow("Visualized Matrix", image);  // Show the image on the screen
    cv::waitKey(0);  // Wait for a key press
}

int main() {
    const int rows = 320;  // Height of the image
    const int cols = 480;  // Width of the image
    Eigen::MatrixXi map_matrix(rows, cols);  // Create an Eigen integer matrix

    std::ifstream matrix_file("/home/kgw/hive_ws/src/hive_master/image/transformed_image_matrix.txt");  // Matrix data file
    if (!matrix_file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
        return -1;
    }

    // Read the matrix data from the file into the Eigen matrix
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            matrix_file >> map_matrix(r, c);
        }
    }
    matrix_file.close();

    // Create maps with expanded colors
    Eigen::MatrixXi expanded = expandAllColors(map_matrix, 20);
    // Create maps for each robot color
    Eigen::MatrixXi map1 = makeNewMap(map_matrix, 2, 20);
    Eigen::MatrixXi map2 = makeNewMap(map_matrix, 3, 20);
    Eigen::MatrixXi map3 = makeNewMap(map_matrix, 4, 20);

    // Convert matrices to images
    cv::Mat img_expanded = matrixToImage(expanded);
    cv::Mat img_map1 = matrixToImage(map1);
    cv::Mat img_map2 = matrixToImage(map2);
    cv::Mat img_map3 = matrixToImage(map3);

    // Combine images horizontally to form two rows
    cv::Mat top_row, bottom_row;
    cv::hconcat(std::vector<cv::Mat>{img_expanded, img_map1}, top_row);
    cv::hconcat(std::vector<cv::Mat>{img_map2, img_map3}, bottom_row);

    // Combine the two rows vertically to form the final image
    cv::Mat final_image;
    cv::vconcat(std::vector<cv::Mat>{top_row, bottom_row}, final_image);

    // Display the merged image
    cv::imshow("Comparison of Maps", final_image);
    cv::waitKey(0);

    return 0;
}
