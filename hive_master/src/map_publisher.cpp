#include <rclcpp/rclcpp.hpp> // ROS 2 C++ 클라이언트 라이브러리를 포함합니다.
#include <nav_msgs/msg/occupancy_grid.hpp> // ROS 2 메시지 타입 중 OccupancyGrid를 사용하기 위해 포함합니다.
#include <Eigen/Dense> // Eigen 라이브러리, 주로 매트릭스 계산에 사용됩니다.
#include <fstream> // 파일 입출력을 위한 표준 라이브러리 포함
#include <vector> // std::vector 컨테이너 사용을 위해 포함
#include <string> // std::string 사용을 위해 포함
#include <tf2_ros/static_transform_broadcaster.h> // 정적 변환을 발행하기 위한 라이브러리
#include <geometry_msgs/msg/transform_stamped.hpp> // 변환 메시지를 사용하기 위해 포함

using std::placeholders::_1;

// MapPublisher 클래스는 ROS 2 노드로, 맵 데이터를 퍼블리시하는 기능을 수행합니다.
class MapPublisher : public rclcpp::Node {
public:
    // 노드 생성자
    MapPublisher() : Node("map_publisher") {
        // 노드에서 사용할 파라미터들을 선언합니다.
        this->declare_parameter<std::string>("map_file", "/home/kgw/hive_ws/src/hive_master/image/transformed_image_matrix.txt");
        this->declare_parameter<int>("rows", 320);
        this->declare_parameter<int>("cols", 480);
        this->declare_parameter<float>("resolution", 0.05);

        std::string map_file;
        int rows, cols;
        float resolution;

        // 파라미터 값을 가져옵니다.
        this->get_parameter("map_file", map_file);
        this->get_parameter("rows", rows);
        this->get_parameter("cols", cols);
        this->get_parameter("resolution", resolution);

        // 텍스트 파일에서 맵 데이터를 로드합니다.
        map_matrix_ = loadMapFromTxt(map_file, rows, cols);
        if (map_matrix_.size() == 0) {
            RCLCPP_ERROR(this->get_logger(), "Map loading failed.");
            rclcpp::shutdown();
            return;
        }

        // 로드된 데이터를 ROS 메시지로 변환합니다.
        grid_ = convertToOccupancyGrid(map_matrix_, resolution);
        // OccupancyGrid 메시지를 퍼블리시할 퍼블리셔를 설정합니다.
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
        // 0.1초마다 맵 데이터를 퍼블리시하도록 타이머를 설정합니다.
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MapPublisher::publishMap, this));

        // 정적 변환을 발행합니다.
        publishStaticTransform();
    }

private:
    // 텍스트 파일로부터 맵 데이터를 읽어 Eigen::MatrixXi 형태로 반환하는 함수
    Eigen::MatrixXi loadMapFromTxt(const std::string& filename, int rows, int cols) {
        Eigen::MatrixXi map(rows, cols);
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
            return map; // 파일 열기 실패 시 빈 맵 반환
        }

        int value;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                file >> value;
                map(i, j) = value; // 파일에서 읽은 값을 맵에 저장
            }
        }
        file.close();
        return map;
    }

    // Eigen::MatrixXi 데이터를 nav_msgs::msg::OccupancyGrid 형태로 변환하는 함수
    nav_msgs::msg::OccupancyGrid convertToOccupancyGrid(const Eigen::MatrixXi& map, float resolution) {
        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp = this->now();
        grid.header.frame_id = "map";
        grid.info.resolution = resolution;
        grid.info.width = map.cols();
        grid.info.height = map.rows();
        grid.info.origin.position.x = 0.0;
        grid.info.origin.position.y = 0.0;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;

        grid.data.resize(grid.info.width * grid.info.height);
        for (uint i = 0; i < grid.info.height; i++) {
            for (uint j = 0; j < grid.info.width; j++) {
                grid.data[i * grid.info.width + j] = map(i, j);
            }
        }
        return grid;
    }

    // 정적 변환을 발행하는 함수
    void publishStaticTransform() {
        static tf2_ros::StaticTransformBroadcaster static_broadcaster(this);
        geometry_msgs::msg::TransformStamped static_transformStamped;

        static_transformStamped.header.stamp = this->now();
        static_transformStamped.header.frame_id = "world";
        static_transformStamped.child_frame_id = "map";
        static_transformStamped.transform.translation.x = 0.0;
        static_transformStamped.transform.translation.y = 0.0;
        static_transformStamped.transform.translation.z = 0.0;
        static_transformStamped.transform.rotation.x = 0.0;
        static_transformStamped.transform.rotation.y = 0.0;
        static_transformStamped.transform.rotation.z = 0.0;
        static_transformStamped.transform.rotation.w = 1.0;

        static_broadcaster.sendTransform(static_transformStamped);
    }

    // 맵 데이터를 주기적으로 발행하는 타이머 콜백 함수
    void publishMap() {
        publisher_->publish(grid_);
    }

    rclcpp::TimerBase::SharedPtr timer_; // 타이머
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_; // 퍼블리셔
    nav_msgs::msg::OccupancyGrid grid_; // 퍼블리시할 메시지
    Eigen::MatrixXi map_matrix_; // 맵 데이터 저장 변수
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv); // ROS 2 초기화
    rclcpp::spin(std::make_shared<MapPublisher>()); // 노드 실행
    rclcpp::shutdown(); // ROS 2 종료
    return 0;
}
