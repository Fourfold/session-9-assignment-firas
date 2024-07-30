#include "closest_wall_server.h"

ClosestWallServer::ClosestWallServer() : Node("closest_wall_server") {
    service_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>
        ("/scan", 10, std::bind(&ClosestWallServer::scan_callback, this, std::placeholders::_1));
    service = this->create_service<custom_interfaces::srv::FindClosestWall>
        ("find_closest_wall", std::bind(&ClosestWallServer::find_closest_wall_callback, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, service_cb_group);
    scanning = false;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FindClosestWall server started. Ready for requests.");
}

void ClosestWallServer::scan_callback(const sensor_msgs::msg::LaserScan & data) {
    if (!scanning) {
        return;
    } else {
        scan_data = &data.ranges;
        scanning = false;
    }
}

void ClosestWallServer::find_closest_wall_callback(const std::shared_ptr<custom_interfaces::srv::FindClosestWall::Request> request,
    std::shared_ptr<custom_interfaces::srv::FindClosestWall::Response> response) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FindClosestWall request received.");
        scanning = true;
        while (scanning) {}
        bool rotateRight = getRotationDirection();

        auto msg = geometry_msgs::msg::Twist();
        msg.angular = geometry_msgs::msg::Vector3();
        if (rotateRight) {
            msg.angular.z = -0.25;
        } else {
            msg.angular.z = 0.25;
        }
        publisher->publish(msg);
        
        bool done = false;
        while (!done) {
            scanning = true;
            while (scanning) {}
            done = isCorrectDirection();
        }

        msg.angular.z = 0.0;
        publisher->publish(msg);

        response->successful = true;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FindClosestWall response sent.");
    }

bool ClosestWallServer::getRotationDirection() {
    int closestIndex = 0;
    float closestDistance = scan_data->at(360-4) + scan_data->at(360-3) + scan_data->at(360-2) + scan_data->at(360-1) + scan_data->at(0) + scan_data->at(1) + scan_data->at(2) + scan_data->at(3) + scan_data->at(4);
    for (int i = 1; i < 40; i++) {
        float sum = 0.0;
        for (int j = 9*i-4; i < 9*(i+1)-4; i++) {
            sum += scan_data->at(j);
        }
        if (sum < closestDistance) {
            closestIndex = i;
            closestDistance = sum;
        }
    }
    if (closestIndex < 20) {
        return false;
    } else {
        return true;
    }
}

bool ClosestWallServer::isCorrectDirection() {
    float forwardDistance = scan_data->at(360-4) + scan_data->at(360-3) + scan_data->at(360-2) + scan_data->at(360-1) + scan_data->at(0) + scan_data->at(1) + scan_data->at(2) + scan_data->at(3) + scan_data->at(4);
    for (int i = 1; i < 40; i++) {
        float sum = 0.0;
        for (int j = 9*i-4; i < 9*(i+1)-4; i++) {
            sum += scan_data->at(j);
        }
        if (sum < forwardDistance) {
            return false;
        }
    }
    return true;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto server_node = std::make_shared<ClosestWallServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(server_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}