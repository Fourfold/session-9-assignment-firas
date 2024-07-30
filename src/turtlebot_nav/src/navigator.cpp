#include "navigator.h"

using namespace std::chrono_literals;

Navigator::Navigator() : Node("navigator_node") {
    publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    client = this->create_client<custom_interfaces::srv::FindClosestWall>("find_closest_wall");
    request = std::make_shared<custom_interfaces::srv::FindClosestWall::Request>();
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }
    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received response from server.");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service.");
    }
  
    subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>
    ("/scan", 10, std::bind(&Navigator::scan_callback, this, std::placeholders::_1));
};

void Navigator::scan_callback(const sensor_msgs::msg::LaserScan & data) {
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigator>());
  rclcpp::shutdown();
  return 0;
}