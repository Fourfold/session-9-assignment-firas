#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/find_closest_wall.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ClosestWallServer : public rclcpp::Node {
public:
    ClosestWallServer();

private:
    void scan_callback(const sensor_msgs::msg::LaserScan & data);
    void find_closest_wall_callback(const std::shared_ptr<custom_interfaces::srv::FindClosestWall::Request> request,
          std::shared_ptr<custom_interfaces::srv::FindClosestWall::Response> response)
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber;
    rclcpp::Service<custom_interfaces::srv::FindClosestWall>::SharedPtr service =
        node->create_service<custom_interfaces::srv::FindClosestWall>("find_closest_wall", &find_closest_wall_callback);
    bool scanning;
    float* scan_data;
};