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

#include "ras_bt_framework/PrimitiveBehavior.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ras_interfaces/srv/play_path.hpp" 
#include "rclcpp/logging.hpp"
#include <ras_interfaces/srv/status_log.hpp>    


namespace ras_bt_framework
{

class ExecuteTrajectory : public PrimitiveBehavior {
public:
    int instruction_no;
    ExecuteTrajectory(const std::string& name, const BT::NodeConfig& config)
    : PrimitiveBehavior(name, config)
    {
        // Initialize other members here, like the ROS node
        node_ = rclcpp::Node::make_shared("execute_trajectory_node");
        play_traj = node_->create_client<ras_interfaces::srv::PlayPath>("/play_trajectory");
        client_log = node_->create_client<ras_interfaces::srv::StatusLog>("/traj_status");

    }

    ~ExecuteTrajectory() {}
    
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("sequence") };
    }
    
   virtual BT::NodeStatus tick() override {
    std::cout << ("ExecuteTrajectory") << std::endl;

    auto msg = getInput<std::string>("sequence");

    auto request = std::make_shared<ras_interfaces::srv::PlayPath::Request>();

    request->unique_id = msg.value();

    request->topic_name = "trajectory_topic"; 

    auto result_future = play_traj->async_send_request(
            request, std::bind(&ExecuteTrajectory::play_traj_response, this,
                                std::placeholders::_1));  

    if (rclcpp::spin_until_future_complete(node_, result_future) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {

    auto request = std::make_shared<ras_interfaces::srv::StatusLog::Request>();
    request->traj_status = "SUCCESS"; 
    request->gripper_status = false;
    request->current_traj = stoi(msg.value()); 

    auto result_future = client_log->async_send_request(
    request, std::bind(&ExecuteTrajectory::log_response, this,
                    std::placeholders::_1));
    return BT::NodeStatus::SUCCESS;
    }
    
   }

    void play_traj_response(rclcpp::Client<ras_interfaces::srv::PlayPath>::SharedFuture future) {
    }

    void log_response(rclcpp::Client<ras_interfaces::srv::StatusLog>::SharedFuture future) {
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<ras_interfaces::srv::PlayPath>::SharedPtr play_traj;
    rclcpp::Client<ras_interfaces::srv::StatusLog>::SharedPtr client_log;
};

}
