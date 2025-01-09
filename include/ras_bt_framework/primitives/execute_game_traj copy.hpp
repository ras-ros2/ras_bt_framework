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

#include "../PrimitiveBehavior.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ras_interfaces/srv/play_path.hpp" 
#include "rclcpp/logging.hpp"
#include <ras_interfaces/srv/status_log.hpp>    


namespace ras_bt_framework
{

NEW_PRIMITIVE_DECL(ExecuteGameTrajectoryNode)
    public:
    void initialize() override
    {
        node_ = rclcpp::Node::make_shared("execute_game_traj_node");
        play_traj = node_->create_client<ras_interfaces::srv::PlayPath>("/play_trajectory");
        client_log = node_->create_client<ras_interfaces::srv::StatusLog>("/traj_status");

        // Subscribe to the selected cell number topic
        cell_number_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
            "/selected_cell_number", 10,
            std::bind(&ExecuteGameTrajNode::cell_number_callback, this, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(), "ExecuteGameTrajNode initialized and subscribed to /selected_cell_number.");

    }

    int instruction_no;
    ~ExecuteGameTrajectoryNode() {}
    
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("") };
    }
    
   virtual BT::NodeStatus tick() override {
    std::cout << ("ExecuteGameTrajectoryNode") << std::endl;

    auto request = std::make_shared<ras_interfaces::srv::PlayPath::Request>();

    request->unique_id = std::to_string(selected_cell_);

    request->topic_name = "trajectory_topic"; 

    auto result_future = play_traj->async_send_request(
            request, std::bind(&ExecuteGameTrajectoryNode::play_traj_response, this,
                                std::placeholders::_1));  

    if (rclcpp::spin_until_future_complete(node_, result_future) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {

    auto request = std::make_shared<ras_interfaces::srv::StatusLog::Request>();
    request->traj_status = "SUCCESS"; 
    request->gripper_status = false;
    request->current_traj = stoi(msg.value()); 

    auto result_future = client_log->async_send_request(
    request, std::bind(&ExecuteGameTrajectoryNode::log_response, this,
                    std::placeholders::_1));
    return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
   }

    void play_traj_response(rclcpp::Client<ras_interfaces::srv::PlayPath>::SharedFuture future) {
    }

    void log_response(rclcpp::Client<ras_interfaces::srv::StatusLog>::SharedFuture future) {
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<ras_interfaces::srv::PlayPath>::SharedPtr play_traj;
    rclcpp::Client<ras_interfaces::srv::StatusLog>::SharedPtr client_log;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cell_number_sub_;
    int selected_cell_; // Cell number received from the topic
    void cell_number_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        selected_cell_ = msg->data;
        RCLCPP_INFO(ros_node_->get_logger(), "Received cell number: %d", selected_cell_);
    }
    
END_PRIMITIVE_DECL
};

