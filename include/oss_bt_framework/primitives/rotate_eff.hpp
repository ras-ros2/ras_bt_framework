#pragma once

#include "oss_bt_framework/PrimitiveBehavior.hpp"
#include "rclcpp/rclcpp.hpp"
#include "oss_interfaces/srv/rotate_effector.hpp"
#include <iostream>
#include "rclcpp/logging.hpp" // For ROS_INFO

namespace oss_bt_framework
{

    class RotateEffector : public PrimitiveBehavior
    {
    public:
        RotateEffector(const std::string &name, const BT::NodeConfig &config)
            : PrimitiveBehavior(name, config)
        {
            node_ = rclcpp::Node::make_shared("rotate_effector_node");
            rotate_eff_client_ = node_->create_client<oss_interfaces::srv::RotateEffector>("/rotate_effector");
        }

        ~RotateEffector() {}

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<double>("rotation_angle")};
        }

        virtual BT::NodeStatus tick() override
        {
            std::cout << "RotateEffector tick()" << std::endl;

            // Retrieve input angle
            auto angle = getInput<double>("rotation_angle");
            if (!angle)
            {
                throw BT::RuntimeError("Missing required input [rotation_angle]: ", angle.error());
            }

            auto request = std::make_shared<oss_interfaces::srv::RotateEffector::Request>();
            request->rotation_angle = angle.value();

            std::cout << "Sending rotation angle: " << request->rotation_angle << std::endl;

            auto result_future = rotate_eff_client_->async_send_request(
                request, std::bind(&RotateEffector::rotate_eff_response, this, std::placeholders::_1));

            // Spin and wait for the result
            if (rclcpp::spin_until_future_complete(node_, result_future) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto result = result_future.get();
                if (result->success)
                {
                    RCLCPP_INFO(node_->get_logger(), "Rotation successful.");
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "Rotation failed.");
                    return BT::NodeStatus::FAILURE;
                }
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Service call failed.");
                return BT::NodeStatus::FAILURE;
            }
        }

        void rotate_eff_response(rclcpp::Client<oss_interfaces::srv::RotateEffector>::SharedFuture future)
        {
            // Handle the response if needed (not required here for simplicity)
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<oss_interfaces::srv::RotateEffector>::SharedPtr rotate_eff_client_;
    };

} // namespace oss_bt_framework
