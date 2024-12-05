#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "oss_interfaces/srv/traj_action.hpp"

using TrajAction = oss_interfaces::srv::TrajAction;

using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

namespace behavior_tree_plugins
{

class GetTrajectory : public BT::SyncActionNode
{
public:
    GetTrajectory(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), initial_total_actions_(-1)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

        service_ = node_->create_service<TrajAction>(
            "action_traj", std::bind(&GetTrajectory::handle_traj_action, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(node_->get_logger(), "Trajectory server ready.");

        // Initialize flag_action to true
        config.blackboard->set("flag_action", true);

        // Set initial_total_actions in the blackboard
        config.blackboard->set("initial_total_actions", initial_total_actions_);

        // Create exp_completed variable and set it to true
        config.blackboard->set("exp_completed", true);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        auto blackboard = config().blackboard;

        std::cout << "GetTraj Node Called" << std::endl;

        config().blackboard->get("instructions", instructions_);

        bool exp_completed;
        config().blackboard->get("exp_completed", exp_completed);

        if (instructions_.empty() && exp_completed == true)
        {
            RCLCPP_ERROR(node_->get_logger(), "No instructions or trajectories received yet.");
            return BT::NodeStatus::FAILURE;
        }

        if (initial_total_actions_ == -1)
        {
            initial_total_actions_ = instructions_.size;
            config().blackboard->set("initial_total_actions", initial_total_actions_);
            RCLCPP_INFO(node_->get_logger(), "Initial total actions: %d", initial_total_actions_);
        }

        config().blackboard->set("instructions", instructions_);
        total_instructions_ = instructions_.size;
        std::cout << "Total Instructions : " << total_instructions_ << std::endl;
        config().blackboard->set("total_instructions", total_instructions_);

        config().blackboard->set("gripper_status", false);

        std::cout << "Instructions: ";
        for (const auto& instruction : instructions_)
        {
            std::cout << instruction << " ";
        }
        std::cout << std::endl;

        std::vector<int> numbered_traj_list;
        std::vector<int> status_list;
        int traj_count = 0, status_count = 0;

        for (const auto& instruction : instructions_)
        {
            if (instruction == "traj")
            {
                numbered_traj_list.push_back(traj_count);
                traj_count++;
            }
        }

        config().blackboard->set("trajectories", numbered_traj_list);
        std::cout << "Sequence: ";
        for (const auto& num : numbered_traj_list)
        {
            std::cout << num << " ";
        }
        std::cout << std::endl;

        for (const auto& status : instructions_)
        {
            if (status == "pick" || status == "drop")
            {
                status_list.push_back(status_count);
                status_count++;
            }
        }

        config().blackboard->set("status", status_list);
        std::cout << "Status: ";
        for (const auto& num : status_list)
        {
            std::cout << num << " ";
        }
        std::cout << std::endl;

        int current_traj = 0;
        blackboard->set("current_traj", current_traj);

        return BT::NodeStatus::SUCCESS;
    }

private:
    void handle_traj_action(const std::shared_ptr<TrajAction::Request> request,
                            std::shared_ptr<TrajAction::Response> response)
    {
        RCLCPP_INFO(node_->get_logger(), "Received TrajAction request");

        instructions_ = request->instructions;
        trajectories_ = request->traj;
        config().blackboard->set("exp_completed", false);
        initial_total_actions_ = -1;
        response->success = true;

        total_instructions_ = instructions_.size;
        std::cout << "Total Instructions RESET: " << total_instructions_ << std::endl;

        config().blackboard->set("instructions", instructions_);
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Service<TrajAction>::SharedPtr service_;
    std::vector<std::string> instructions_;
    std::vector<JointTrajectory> trajectories_;
    int initial_total_actions_;
    int total_instructions_, total_actions = 0;
};

} // namespace behavior_tree_plugins

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree_plugins::GetTrajectory>("GetTrajectory");
}
