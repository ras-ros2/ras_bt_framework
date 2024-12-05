#include <iostream>
#include <vector>
#include <chrono>
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "oss_interfaces/srv/traj_sec.hpp"
#include "oss_interfaces/srv/status_log.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace behavior_tree_plugins {

class Traj : public BT::SyncActionNode {
public:
    Traj(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        client_ = node_->create_client<oss_interfaces::srv::TrajSec>("/trajectory_sequence");
        logClient_ = node_->create_client<oss_interfaces::srv::StatusLog>("traj_status");
    }

    static BT::PortsList providedPorts() {
        return {};
    }

    BT::NodeStatus tick() override {
        bool flag_action;
        auto blackboard = config().blackboard;

        blackboard->get("flag_action", flag_action);

        std::vector<std::string> instructions;
        std::vector<int> trajectories;

        blackboard->get("instructions", instructions);
        blackboard->get("trajectories", trajectories);

        if (!instructions.empty()) {
            if (instructions[0] == "traj" && flag_action) {
                bool gripper_status;
                blackboard->get("gripper_status", gripper_status);

                RCLCPP_INFO(node_->get_logger(), "Gripper Status: %s", gripper_status ? "true" : "false");

                if (!trajectories.empty()) {
                    RCLCPP_INFO(node_->get_logger(), "trajectories[0]: %d", trajectories[0]);
                    auto result = send_request(trajectories[0]);
                    if (result == rclcpp::FutureReturnCode::SUCCESS) {
                        RCLCPP_INFO(node_->get_logger(), "Request was successful.");
                    } else {
                        RCLCPP_ERROR(node_->get_logger(), "Request failed.");
                        return BT::NodeStatus::FAILURE;
                    }
                    trajectories.erase(trajectories.begin());
                }

                instructions.erase(instructions.begin());
                blackboard->set("instructions", instructions);
                blackboard->set("trajectories", trajectories);
                blackboard->set("flag_action", false);

                int current_traj;
                blackboard->get("current_traj", current_traj);

                send_status_log("SUCCESS", gripper_status, current_traj);

                RCLCPP_INFO(node_->get_logger(), "########Traj Updated instructions:");
                std::cout << "Instructions: ";
                for (const auto& instruction : instructions) {
                    std::cout << instruction << " ";
                }
                std::cout << std::endl;

                std::cout << "Sequence: ";
                for (const auto& seq : trajectories) {
                    std::cout << seq << " ";
                }
                std::cout << std::endl;
            }

            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::FutureReturnCode send_request(int trajectory) {
        auto request = std::make_shared<oss_interfaces::srv::TrajSec::Request>();
        request->data = trajectory;

        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return rclcpp::FutureReturnCode::INTERRUPTED;
            }
            RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
        }

        auto future_result = client_->async_send_request(request);
        auto status = rclcpp::spin_until_future_complete(node_, future_result);
        if (status == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future_result.get();
            if (response->success) {
                RCLCPP_INFO(node_->get_logger(), "Service call succeeded.");
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Service call failed.");
                return rclcpp::FutureReturnCode::INTERRUPTED;
            }
        }
        return status;
    }

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

        logClient_->async_send_request(request, response_received_callback);
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<oss_interfaces::srv::TrajSec>::SharedPtr client_;
    rclcpp::Client<oss_interfaces::srv::StatusLog>::SharedPtr logClient_;
};

} // namespace behavior_tree_plugins

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<behavior_tree_plugins::Traj>("Traj");
}
