/*
 * 
 * Copyright (C) 2024 Sahil Lathwal
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
#include "ras_interfaces/srv/update_markers.hpp"
#include "rclcpp/logging.hpp"  // For logging

namespace ras_bt_framework
{

// Define a new PrimitiveBehavior named "UpdateRealInventory"
NEW_PRIMITIVE_DECL(UpdateRealInventory)
public:
    void initialize() override
    {
        // Create local ROS node
        node_ = rclcpp::Node::make_shared("update_real_inventory_node");

        // Create a client for the /update_markers service (type: ras_interfaces::srv::UpdateMarkers)
        update_markers_client_ =
            node_->create_client<ras_interfaces::srv::UpdateMarkers>("/update_markers");

        // Optionally wait a bit for the service to be available
        if (!update_markers_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_WARN(
                node_->get_logger(),
                "Service '/update_markers' not available after waiting. Will proceed anyway...");
        }
    }

    ~UpdateRealInventory() {}

    // No input ports needed because the service doesn't require request data
    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(node_->get_logger(), "UpdateRealInventory: Sending request to /update_markers");

        // Prepare an empty request (our .srv has no input fields in the request part)
        auto request = std::make_shared<ras_interfaces::srv::UpdateMarkers::Request>();

        // Send async request and block until done (spin_until_future_complete)
        auto result_future = update_markers_client_->async_send_request(
            request,
            std::bind(&UpdateRealInventory::onResponse, this, std::placeholders::_1)
        );

        // Wait for the service to complete
        if (rclcpp::spin_until_future_complete(node_, result_future)
            == rclcpp::FutureReturnCode::SUCCESS)
        {
            // Check the result
            auto result = result_future.get();
            if (result->success)
            {
                RCLCPP_INFO(node_->get_logger(),
                            "UpdateMarkes succeeded: %s", result->message.c_str());
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(),
                            "UpdateMarkers failed: %s", result->message.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        else
        {
            // Future not completed, or some error
            RCLCPP_ERROR(node_->get_logger(),
                         "Service call '/update_markers' failed or timed out");
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    // (Optional) callback if you want to handle the async response
    void onResponse(rclcpp::Client<ras_interfaces::srv::UpdateMarkers>::SharedFuture future)
    {
        auto result = future.get();
        RCLCPP_INFO(node_->get_logger(),
                    "UpdateRealInventory received response: success=%d, message=%s",
                    result->success, result->message.c_str());
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<ras_interfaces::srv::UpdateMarkers>::SharedPtr update_markers_client_;

END_PRIMITIVE_DECL

} // end namespace ras_bt_framework
