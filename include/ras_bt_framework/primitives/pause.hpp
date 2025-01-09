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
#include "rclcpp/logging.hpp"
#include <std_msgs/msg/string.hpp>
#include <json/json.h> 

namespace ras_bt_framework
{

NEW_PRIMITIVE_DECL(Pause)
    public:
    void initialize() override
    {
        node_ = rclcpp::Node::make_shared("pause");

         // Subscribe to the inventory state topic
        inventory_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/inventory/state", 10,
            std::bind(&PauseNode::inventory_callback, this, std::placeholders::_1));

        prev_empty_cells = 0;
        current_empty_cells = 0;
    }

    ~Pause() {}
    
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("") };
    }
    
   virtual BT::NodeStatus tick() override {
    std::cout << ("Pause") << std::endl;

     // If the current empty cells count has reduced, human has made a move
    if (current_empty_cells < prev_empty_cells)
    {
        prev_empty_cells = current_empty_cells; // Update the previous state
        RCLCPP_INFO(node_->get_logger(), "Human move detected, resuming behavior tree.");
        return BT::NodeStatus::SUCCESS;
    }

    // On first run, initialize the previous empty cells count
    if (prev_empty_cells == 0)
    {
        prev_empty_cells = current_empty_cells;
    }

    RCLCPP_INFO(node_->get_logger(), "Waiting for human move... Current empty cells: %d", current_empty_cells);
    return BT::NodeStatus::FAILURE;
   }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr inventory_sub_;
    int prev_empty_cells;
    int current_empty_cells;
    
     void inventory_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        try
        {
            // Parse the JSON data to extract total_empty_cells
            Json::Value root;
            Json::Reader reader;
            if (!reader.parse(msg->data, root))
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to parse inventory message: %s", msg->data.c_str());
                return;
            }

            // Extract total_empty_cells from the parsed JSON
            if (root["board"].isMember("total_empty_cells"))
            {
                current_empty_cells = root["board"]["total_empty_cells"].asInt();
                RCLCPP_INFO(node_->get_logger(), "Updated total empty cells: %d", current_empty_cells);
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "total_empty_cells not found in inventory message.");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Error processing inventory message: %s", e.what());
        }
    }
END_PRIMITIVE_DECL
};

