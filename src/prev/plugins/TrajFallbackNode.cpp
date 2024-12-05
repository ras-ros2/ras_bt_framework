#include <iostream>
#include <vector>
#include <chrono>

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "oss_interfaces/srv/status_log.hpp"
#include "std_srvs/srv/set_bool.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace behavior_tree_plugins {

class TrajFallback : public BT::SyncActionNode {
public:
    TrajFallback(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        logClient_ = node_->create_client<oss_interfaces::srv::StatusLog>("traj_status");
        fallbackClient_ = node_->create_client<std_srvs::srv::SetBool>("restart_arm");
    }

    static BT::PortsList providedPorts() {
        return {};
    }

    BT::NodeStatus tick() override {
        auto blackboard = config().blackboard;

        std::vector<std::string> instructions_;
        std::vector<int> trajectories;

        blackboard->get("instructions", instructions_);
        blackboard->get("trajectories", trajectories);

        blackboard->set("total_actions", 0);
        RCLCPP_INFO(node_->get_logger(), "Experiment Interrupted. Calling server again");
        instructions_.clear();
        blackboard->set("instructions", instructions_);
        blackboard->set("exp_completed", true);
        blackboard->set("flag_action", true);

        bool gripper_status;
        blackboard->get("gripper_status", gripper_status);

        int current_traj;
        blackboard->get("current_traj", current_traj);
        blackboard->set("current_traj", current_traj);

        send_status_log("FAILED", gripper_status, current_traj);
        send_set_bool_request(true);

        return BT::NodeStatus::FAILURE;
    }

private:
    void send_status_log(const std::string& traj_status, bool gripper_status, int32_t current_traj) {
        auto request = std::make_shared<oss_interfaces::srv::StatusLog::Request>();
        request->traj_status = traj_status;
        request->gripper_status = gripper_status;
        request->current_traj = current_traj;

        while (!logClient_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
        }

        auto response_received_callback = [this](rclcpp::Client<oss_interfaces::srv::StatusLog>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(node_->get_logger(), "Status log request was successful.");
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Status log request failed.");
            }
        };

        auto future_result = logClient_->async_send_request(request, response_received_callback);
    }

    void send_set_bool_request(bool data) {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = data;

        while (!fallbackClient_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the SetBool service. Exiting.");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "SetBool service not available, waiting again...");
        }

        auto response_received_callback = [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(node_->get_logger(), "SetBool request was successful.");
            } else {
                RCLCPP_ERROR(node_->get_logger(), "SetBool request failed.");
            }
        };

        auto future_result = fallbackClient_->async_send_request(request, response_received_callback);
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<oss_interfaces::srv::StatusLog>::SharedPtr logClient_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr fallbackClient_;
};

} // namespace behavior_tree_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<behavior_tree_plugins::TrajFallback>("TrajFallback");
}

