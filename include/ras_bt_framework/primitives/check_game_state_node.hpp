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
#include <map>
#include <string>

namespace ras_bt_framework
{

NEW_PRIMITIVE_DECL(CheckGameStateNode)
    public:
    void initialize() override
    {
        node_ = rclcpp::Node::make_shared("CheckGameStateNode");

         // Subscribe to the inventory state topic
        inventory_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/inventory/state", 10,
            std::bind(&CheckGameStateNodeNode::inventory_callback, this, std::placeholders::_1));
    }

    ~CheckGameStateNode() {}
    
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("") };
    }
    
   virtual BT::NodeStatus tick() override {
    std::cout << ("CheckGameStateNode") << std::endl;

    // Ensure board state is valid
    if (board_state_.empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "Board state is empty! Cannot check game state.");
        return BT::NodeStatus::FAILURE;
    }

    // Check for win, loss, or draw
    int result = check_winner(board_state_);
    if (result == 1)
    {
        RCLCPP_INFO(node_->get_logger(), "Robot wins!");
        return BT::NodeStatus::SUCCESS;
    }
    else if (result == -1)
    {
        RCLCPP_INFO(node_->get_logger(), "Human wins!");
        return BT::NodeStatus::SUCCESS;
    }
    else if (is_board_full(board_state_))
    {
        RCLCPP_INFO(node_->get_logger(), "Game ends in a draw.");
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE; // Game is still ongoing
   }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr inventory_sub_;

    std::map<int, std::string> board_state_; // Map of cell index to status ("empty", "filled_robot", "filled_human")

    void inventory_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        try
        {
            // Parse the JSON data to extract the board state
            Json::Value root;
            Json::Reader reader;
            if (!reader.parse(msg->data, root))
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to parse inventory message: %s", msg->data.c_str());
                return;
            }

            // Extract the board state
            if (root["board"].isMember("cells"))
            {
                const Json::Value &cells = root["board"]["cells"];
                board_state_.clear();

                for (const auto &cell : cells.getMemberNames())
                {
                    int cell_index = std::stoi(cell);
                    std::string cell_status = cells[cell]["status"].asString();
                    board_state_[cell_index] = cell_status;
                }

                RCLCPP_INFO(node_->get_logger(), "Board state updated from inventory.");
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "Board cells not found in inventory message.");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Error processing inventory message: %s", e.what());
        }
    }

    int check_winner(const std::map<int, std::string> &board_state)
    {
        // Winning combinations
        std::vector<std::vector<int>> win_combinations = {
            {1, 2, 3}, {4, 5, 6}, {7, 8, 9}, // Rows
            {1, 4, 7}, {2, 5, 8}, {3, 6, 9}, // Columns
            {1, 5, 9}, {3, 5, 7}             // Diagonals
        };

        for (const auto &combo : win_combinations)
        {
            if (board_state.at(combo[0]) == "filled_robot" &&
                board_state.at(combo[1]) == "filled_robot" &&
                board_state.at(combo[2]) == "filled_robot")
            {
                return 1; // Robot wins
            }

            if (board_state.at(combo[0]) == "filled_human" &&
                board_state.at(combo[1]) == "filled_human" &&
                board_state.at(combo[2]) == "filled_human")
            {
                return -1; // Human wins
            }
        }

        return 0; // No winner
    }

    bool is_board_full(const std::map<int, std::string> &board_state)
    {
        for (const auto &[cell, status] : board_state)
        {
            if (status == "empty")
            {
                return false;
            }
        }
        return true;
    }
END_PRIMITIVE_DECL
};

