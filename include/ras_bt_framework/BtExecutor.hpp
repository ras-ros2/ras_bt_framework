/*
 * 
 * Copyright (C) 2024 Harsh Davda
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Affero General Public License for more details.
 * 
 * You should have received a copy of the GNU Affero General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 * 
 * For inquiries or further information, you may contact:
 * Harsh Davda
 * Email: info@opensciencestack.org
*/

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
