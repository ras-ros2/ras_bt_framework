/*
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
#include "std_srvs/srv/set_bool.hpp"  // Service used to start/stop video capture

namespace ras_bt_framework
{

NEW_PRIMITIVE_DECL(Capture)
public:
    void initialize() override
    {
        RCLCPP_INFO(node_->get_logger(), "Capture primitive initialized");
        // Create a client for the /realsense_camera_control service
        capture_client = node_->create_client<std_srvs::srv::SetBool>("/realsense_camera_control");
    }

    void destroy() override
    {
    }

    // Declare an input port called "capture_command" of type bool.
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<bool>("capture_command") };
    }

    virtual BT::NodeStatus tick() override
    {
        // Retrieve the input command from the BT blackboard
        auto command = getInput<bool>("capture_command");
        if (!command)
        {
            throw BT::RuntimeError("Missing input [capture_command] in Capture primitive");
        }

        // Create a service request with the provided command value.
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = command.value();

        // Asynchronously send the request.
        auto result_future = capture_client->async_send_request(request);

        // Wait until the service responds.
        if (rclcpp::spin_until_future_complete(node_, result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result_future.get();
            // If the service returns success (and by convention a return code 1), our primitive returns SUCCESS.
            if (response->success)
            {
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                return BT::NodeStatus::FAILURE;
            }
        }
        // If the service call fails (e.g. timeout or error), return FAILURE.
        return BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr capture_client;

END_PRIMITIVE_DECL

}; // namespace ras_bt_framework
