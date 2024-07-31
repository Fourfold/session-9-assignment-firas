#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "custom_interfaces/action/measure_lap_time.hpp"
#include "nav_msgs/msg/odometry.hpp"

using MeasureLapTime = custom_interfaces::action::MeasureLapTime;
using GoalHandleMeasureLapTime = rclcpp_action::ServerGoalHandle<MeasureLapTime>;

class LapTimeServer : public rclcpp::Node {
public:
    LapTimeServer();
private:
    void odometry_callback(const nav_msgs::msg::Odometry & odometry);
    rclcpp_action::Server<MeasureLapTime>::SharedPtr action_server;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MeasureLapTime::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle);
    void execute(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle);
    void publish_elapsed_time();
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber;
    rclcpp::CallbackGroup::SharedPtr service_cb_group;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group;
    rclcpp::TimerBase::SharedPtr timer;
    float x;
    float y;
    float current_time;
    float start_time;
    std::shared_ptr<GoalHandleMeasureLapTime> execute_goal_handle;
    std::shared_ptr<MeasureLapTime::Feedback> feedback_msg;
};