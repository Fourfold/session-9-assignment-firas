#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/find_closest_wall.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class Navigator : public rclcpp::Node {
public:
    Navigator();

private:
    void scan_callback(const sensor_msgs::msg::LaserScan & data);
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber;
    rclcpp::Client<custom_interfaces::srv::FindClosestWall>::SharedPtr client;
    std::shared_ptr<custom_interfaces::srv::FindClosestWall::Request> request;
};