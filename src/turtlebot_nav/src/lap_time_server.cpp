#include "lap_time_server.h"

using MeasureLapTime = custom_interfaces::action::MeasureLapTime;
using GoalHandleMeasureLapTime = rclcpp_action::ServerGoalHandle<MeasureLapTime>;

LapTimeServer::LapTimeServer() : Node("lap_time_server") {
    using namespace std::placeholders;
    service_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    subscription_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->action_server = rclcpp_action::create_server<MeasureLapTime>(
        this,
        "lap_time",
        std::bind(&LapTimeServer::handle_goal, this, _1, _2),
        std::bind(&LapTimeServer::handle_cancel, this, _1),
        std::bind(&LapTimeServer::handle_accepted, this, _1),
        rcl_action_server_get_default_options(),
        service_cb_group
    );
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = subscription_cb_group;
    subscriber = this->create_subscription<nav_msgs::msg::Odometry> (
        "/odom",
        10,
        std::bind(&LapTimeServer::odometry_callback, this, _1),
        subscription_options
    );
    x = 2.0;
    y = 2.0;
    feedback_msg = std::make_shared<MeasureLapTime::Feedback>();
    std::this_thread::sleep_for(std::chrono::seconds(30));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MeasureLapTime server started. Ready for requests.");
}

void LapTimeServer::odometry_callback(const nav_msgs::msg::Odometry & odometry) {
    x = odometry.pose.pose.position.x;
    y = odometry.pose.pose.position.y;
    current_time = odometry.header.stamp.sec + 0.000000001 * odometry.header.stamp.nanosec;
}

rclcpp_action::GoalResponse LapTimeServer::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MeasureLapTime::Goal> goal) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received goal request.");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LapTimeServer::handle_cancel(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received request to cancel goal.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void LapTimeServer::handle_accepted(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&LapTimeServer::execute, this, _1), goal_handle}.detach();
}

void LapTimeServer::execute(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for robot to reach starting point...");
    while (abs(x) > 0.1 && abs(y) > 0.1) continue;
    while (abs(x) < 0.1 || abs(y) < 0.1) continue;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot reached starting point.");

    execute_goal_handle = goal_handle;
    start_time = current_time;

    // TODO: timer
    using namespace std::chrono_literals;
    timer = this->create_wall_timer(
        500ms,
        std::bind(&LapTimeServer::publish_elapsed_time, this),
        timer_cb_group
    );

    for (int i = 0; i < 4; i++) {
        while (abs(x) > 0.1 && abs(y) > 0.1) continue;
        while (abs(x) < 0.1 || abs(y) < 0.1) continue;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot reached finish point.");

    timer = NULL;

    if (rclcpp::ok()) {
        auto result = std::make_shared<MeasureLapTime::Result>();
        result->total_time = current_time - start_time;
        goal_handle->succeed(result);
    }
}

void LapTimeServer::publish_elapsed_time() {
    feedback_msg->elapsed_time = current_time - start_time;
    execute_goal_handle->publish_feedback(feedback_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto server_node = std::make_shared<LapTimeServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(server_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}