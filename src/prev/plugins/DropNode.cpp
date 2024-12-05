#include <iostream>
#include <vector>
#include <chrono>
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "oss_interfaces/srv/status_log.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace behavior_tree_plugins {

class Drop : public BT::SyncActionNode {
public:
    Drop(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        client_ = node_->create_client<std_srvs::srv::SetBool>("gripper_control");
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
        blackboard->get("flag_action", flag_action);

        if (!instructions.empty()) {
            if (instructions[0] == "drop" && flag_action) {
                send_request(false);  // Set gripper state to OFF
                RCLCPP_INFO(node_->get_logger(), "Setting Gripper OFF");
                blackboard->set("gripper_status", false);
                blackboard->set("flag_action", false);  // Set flag_action to false

                instructions.erase(instructions.begin());
                blackboard->set("instructions", instructions);

                RCLCPP_INFO(node_->get_logger(), "########Drop Updated instructions:");
                std::cout << "Instructions: ";
                for (const auto& instruction : instructions) {
                    std::cout << instruction << " ";
                }

                std::cout << "Sequence: ";
                for (const auto& seq : trajectories) {
                    std::cout << seq << " ";
                }
                std::cout << std::endl;

                bool gripper_status;
                blackboard->get("gripper_status", gripper_status);

                int current_traj;
                blackboard->get("current_traj", current_traj);
                current_traj += 1;
                blackboard->set("current_traj", current_traj);

                send_status_log("SUCCESS", gripper_status, current_traj);  // StatusLog Client
            }
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

private:
    void send_request(bool state) {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = state;

        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
        }

        auto response_received_callback = [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(node_->get_logger(), "Request was successful.");
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Request failed.");
            }
        };

        auto future_result = client_->async_send_request(request, response_received_callback);
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

        auto future_result = logClient_->async_send_request(request, response_received_callback);
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
    rclcpp::Client<oss_interfaces::srv::StatusLog>::SharedPtr logClient_;
};

}  // namespace behavior_tree_plugins

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<behavior_tree_plugins::Drop>("Drop");
}
