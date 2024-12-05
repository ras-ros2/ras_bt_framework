#include <chrono>
#include <cstdlib>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "oss_interfaces/srv/traj_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using namespace std::chrono_literals;
using TrajAction = oss_interfaces::srv::TrajAction;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("traj_action_client");
    auto client = node->create_client<TrajAction>("action_traj");

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 1;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for service to appear...");
    }

    auto request = std::make_shared<TrajAction::Request>();
    request->instructions = {"traj", "pick", "drop", "traj"};
    // example sequence for above {0, 1, 2, 3, 4, 5}

    request->traj = {};

    auto result = client->async_send_request(request, [](rclcpp::Client<TrajAction>::SharedFuture future) {
        auto response = future.get();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response received: success = %s", response->success ? "true" : "false");
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
