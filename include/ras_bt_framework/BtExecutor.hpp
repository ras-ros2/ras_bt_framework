#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "ras_interfaces/action/bt_interface.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace ras_bt_framework
{
    class BTExecutor : public rclcpp::Node
    {
        public:
        using BTInterface = ras_interfaces::action::BTInterface;
        using GoalHandle = rclcpp_action::ServerGoalHandle<BTInterface>;
        explicit BTExecutor(std::shared_ptr<BT::BehaviorTreeFactory> bt_factory,const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        bool run_tree_from_xml(std::string file_path);
        private:
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const BTInterface::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
        void execute(const std::shared_ptr<GoalHandle> goal_handle);

        std::shared_ptr<BT::BehaviorTreeFactory> bt_factory_;
        rclcpp_action::Server<BTInterface>::SharedPtr action_server_;

    };    
} // namespace ras_bt_framework
