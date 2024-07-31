#include "lap_time_client.h"

using MeasureLapTime = custom_interfaces::action::MeasureLapTime;
using GoalHandleMeasureLapTime = rclcpp_action::ClientGoalHandle<MeasureLapTime>;

LapTimeClient::LapTimeClient() : Node("lap_time_server") {
    this->action_client = rclcpp_action::create_client<MeasureLapTime>(
        this,
        "lap_time"
    );
}

void LapTimeClient::send_goal() {
    using namespace std::placeholders;
    auto goal_msg = MeasureLapTime::Goal();
    auto send_goal_options = rclcpp_action::Client<MeasureLapTime>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(
        &LapTimeClient::goal_response_callback,
        this,
        _1
    );
    send_goal_options.feedback_callback = std::bind(
        &LapTimeClient::feedback_callback,
        this,
        _1,
        _2
    );
    send_goal_options.result_callback = std::bind(
        &LapTimeClient::result_callback,
        this,
        _1
    );
    this->action_client->async_send_goal(goal_msg, send_goal_options);
}

void LapTimeClient::goal_response_callback(const GoalHandleMeasureLapTime::SharedPtr & goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal rejected.");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted.");
    }
}

void LapTimeClient::feedback_callback(
    GoalHandleMeasureLapTime::SharedPtr,
    const std::shared_ptr<const MeasureLapTime::Feedback> feedback
) {
    std::stringstream ss;
    ss << "Elapsed time: " << 0.001 * round(1000 * feedback->elapsed_time) << " seconds.";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ss.str().c_str());
}

void LapTimeClient::result_callback(const GoalHandleMeasureLapTime::WrappedResult & result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal aborted.");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal canceled.");
            return;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unkown result.");
            return;
    }
    std::stringstream ss;
    ss << "Total lap time: " << 0.001 * round(1000 * result.result->total_time) << " seconds.";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ss.str().c_str());
    rclcpp::shutdown();
}

int main(int argc, char * argv[])
{
    std::this_thread::sleep_for(std::chrono::seconds(30));
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<LapTimeClient>();
    client_node->send_goal();
    rclcpp::spin(client_node);
    return 0;
}