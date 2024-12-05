#include <iostream>
#include <vector>
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

namespace behavior_tree_plugins
{
    class ActionCounter : public BT::SyncActionNode
    {
    public:
        ActionCounter(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
            node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        }

        static BT::PortsList providedPorts()
        {
            return {};
        }

        BT::NodeStatus tick() override
        {   
            std::cout << "Counter node called" << std::endl;
            auto blackboard = config().blackboard;

            int total_actions = 0;
            blackboard->get("total_actions", total_actions);
            total_actions++;
            blackboard->set("total_actions", total_actions);
                // Print total actions
            blackboard->get("total_actions", total_actions);
            std::cout << "Current Action Number incremented to : " << total_actions << std::endl;

            std::vector<std::string> instructions_;

            blackboard->get("instructions", instructions_);
            blackboard->set("flag_action", true) ;

            int initial_total_actions_ = 0;
            blackboard->get("initial_total_actions", initial_total_actions_);

            if (total_actions == initial_total_actions_) 
            {
                blackboard->set("total_actions", 0);
                RCLCPP_INFO(node_->get_logger(), "Resetting total_actions to 0.");
                instructions_.clear(); // Clear old instructions
                blackboard->set("instructions", instructions_);
                blackboard->set("exp_completed", true);
                 blackboard->set("flag_action", true) ;

            }

            return BT::NodeStatus::SUCCESS;
        }

    private:
        rclcpp::Node::SharedPtr node_;
    };
} // namespace behavior_tree_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree_plugins::ActionCounter>("ActionCounter");
}