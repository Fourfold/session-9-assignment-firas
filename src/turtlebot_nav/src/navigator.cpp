#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// #include "custom_interfaces/srv/FindClosestWall.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class Navigator : public rclcpp::Node {
public:
    Navigator() : Node("navigator_node") {
        publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // TODO: client, request, future, spin, reponse, ...
        subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>
        ("/scan", 10, std::bind(&Navigator::scan_callback, this, std::placeholders::_1));
    };

private:
    void scan_callback(const sensor_msgs::msg::LaserScan & data) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear = geometry_msgs::msg::Vector3();
        msg.angular = geometry_msgs::msg::Vector3();
        if (data.ranges[0] < 0.5) {
            msg.linear.x = 0.0;
            msg.angular.z = -0.35;
            }
        else if (data.ranges[25] < 0.55) {
            msg.linear.x = 0.0;
            msg.angular.z = -0.15;
            }
        else {
            msg.linear.x = 0.25;
            msg.angular.z = 0.0;
            }
        publisher->publish(msg);
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigator>());
  rclcpp::shutdown();
  return 0;
}