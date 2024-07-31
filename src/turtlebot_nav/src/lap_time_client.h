#include <memory>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_interfaces/action/measure_lap_time.hpp"

using MeasureLapTime = custom_interfaces::action::MeasureLapTime;
using GoalHandleMeasureLapTime = rclcpp_action::ClientGoalHandle<MeasureLapTime>;

class LapTimeClient : public rclcpp::Node {
public:
    LapTimeClient();
    void send_goal();
private:
    void goal_response_callback(const GoalHandleMeasureLapTime::SharedPtr & goal_handle);
    void feedback_callback(
        GoalHandleMeasureLapTime::SharedPtr,
        const std::shared_ptr<const MeasureLapTime::Feedback> feedback
    );
    void result_callback(const GoalHandleMeasureLapTime::WrappedResult & result);
    rclcpp_action::Client<MeasureLapTime>::SharedPtr action_client;
};