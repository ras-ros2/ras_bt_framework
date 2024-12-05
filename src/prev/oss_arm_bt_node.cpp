#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include <std_srvs/srv/set_bool.hpp>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_msgs/msg/bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "oss_interfaces/srv/traj_action.hpp"
#include "oss_interfaces/srv/status_log.hpp"
#include "oss_interfaces/srv/traj_sec.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

using TrajAction = oss_interfaces::srv::TrajAction;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

class GetTrajectory : public BT::SyncActionNode
{
public:
    GetTrajectory(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

        service_ = node_->create_service<TrajAction>("action_traj", 
            std::bind(&GetTrajectory::handle_traj_action, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(node_->get_logger(), "Trajectory server ready.");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        // RCLCPP_INFO(node_->get_logger(), "Inside tick");
        if (instructions_.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "No instructions or trajectories received yet.");
            return BT::NodeStatus::FAILURE;
        }

        config().blackboard->set("instructions", instructions_);

        config().blackboard->set("gripper_status", false);

        // for (size_t i = 0; i < instructions_.size(); ++i)
        // {
        //     RCLCPP_INFO(node_->get_logger(), "Instruction: %s", instructions_[i].c_str());
        // }

        std::vector<int> numbered_traj_list;
        int traj_count = 0;

        for (const auto& instruction : instructions_) {
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Instruction: %s", instruction.c_str());

            if (instruction == "traj") {
                numbered_traj_list.push_back(traj_count);
                traj_count++;
            }
        }

        config().blackboard->set("trajectories", numbered_traj_list);

        std::cout << "Instructions: ";
        for (const auto& instruction : instructions_) {
            std::cout << instruction << " ";
        }
        std::cout << std::endl;

        std::cout << "Sequence: ";
        for (const auto& num : numbered_traj_list) {
            std::cout << num << " ";
        }
        std::cout << std::endl;

        return BT::NodeStatus::SUCCESS;
    }

private:
    void handle_traj_action(const std::shared_ptr<TrajAction::Request> request,
                            std::shared_ptr<TrajAction::Response> response)
    {
        RCLCPP_INFO(node_->get_logger(), "Received TrajAction request");

        instructions_ = request->instructions;
        trajectories_ = request->traj;

        response->success = true;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Service<TrajAction>::SharedPtr service_;
    std::vector<std::string> instructions_;
    std::vector<JointTrajectory> trajectories_;
};


class Pick : public BT::SyncActionNode
{
public:
    Pick(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        client_ = node_->create_client<std_srvs::srv::SetBool>("gripper_control");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        auto blackboard = config().blackboard;

        std::vector<std::string> instructions;
        std::vector<int> trajectories;

        blackboard->get("instructions", instructions);
        blackboard->get("trajectories", trajectories);

        if (!instructions.empty())
        {
            if(instructions[0] == "pick")
            {
                send_request(true);  // Set gripper state to ON
                blackboard->set("gripper_status", true);
                RCLCPP_INFO(node_->get_logger(), "Setting Gripper ON");

                instructions.erase(instructions.begin());
                // trajectories.erase(trajectories.begin());

                blackboard->set("instructions", instructions);

                blackboard->get("instructions", instructions);

                RCLCPP_INFO(node_->get_logger(), "########Pick Updated instructions:");
                std::cout << "Instructions: ";
                for (const auto& instruction : instructions) {
                    std::cout << instruction << " ";
                }
                std::cout << std::endl;

                std::cout << "Sequence: ";
                for (const auto& seq : trajectories) {
                    std::cout << seq << " ";
                }
                std::cout << std::endl;
            }

            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    void send_request(bool state)
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = state;

        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
        }

        auto response_received_callback = [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(node_->get_logger(), "Request was successful.");
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Request failed.");
            }
        };

        auto future_result = client_->async_send_request(request, response_received_callback);
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
};

class Drop : public BT::SyncActionNode
{
public:
    Drop(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        client_ = node_->create_client<std_srvs::srv::SetBool>("gripper_control");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        auto blackboard = config().blackboard;

        std::vector<std::string> instructions;
        std::vector<int> trajectories;

        blackboard->get("instructions", instructions);
        blackboard->get("trajectories", trajectories);

        if (!instructions.empty())
        {
            if(instructions[0] == "drop")
            {
                send_request(false);  // Set gripper state to OFF
                RCLCPP_INFO(node_->get_logger(), "Setting Gripper OFF");

                blackboard->set("gripper_status", false);

                instructions.erase(instructions.begin());

                blackboard->set("instructions", instructions);

                blackboard->get("instructions", instructions);

                RCLCPP_INFO(node_->get_logger(), "########Drop Updated instructions:");
                std::cout << "Instructions: ";
                for (const auto& instruction : instructions) {
                    std::cout << instruction << " ";
                }
                std::cout << std::endl;

                std::cout << "Sequence: ";
                for (const auto& seq : trajectories) {
                    std::cout << seq << " ";
                }
                std::cout << std::endl;
            }

            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    void send_request(bool state)
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = state;

        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
        }

        auto response_received_callback = [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(node_->get_logger(), "Request was successful.");
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Request failed.");
            }
        };

        auto future_result = client_->async_send_request(request, response_received_callback);
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
};

class Traj : public BT::SyncActionNode
{
public:
    Traj(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        client_ = node_->create_client<oss_interfaces::srv::TrajSec>("/trajectory_sequence");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        auto blackboard = config().blackboard;

        std::vector<std::string> instructions;
        std::vector<int> trajectories;

        blackboard->get("instructions", instructions);
        blackboard->get("trajectories", trajectories);

        if (!instructions.empty())
        {
            if (instructions[0] == "traj")
            {
                bool gripper_status;
                blackboard->get("gripper_status", gripper_status);

                RCLCPP_INFO(node_->get_logger(), "Gripper Status: %s", gripper_status ? "true" : "false");

                if (!trajectories.empty())
                {
                    RCLCPP_INFO(node_->get_logger(), "trajectories[0]: %d", trajectories[0]);
                    send_request(trajectories[0]);
                    trajectories.erase(trajectories.begin());
                }

                instructions.erase(instructions.begin());
                // trajectories.erase(trajectories.begin());

                blackboard->set("instructions", instructions);
                blackboard->set("trajectories", trajectories);

                blackboard->get("instructions", instructions);
                blackboard->get("trajectories", trajectories);

                RCLCPP_INFO(node_->get_logger(), "########Traj Updated instructions:");
                std::cout << "Instructions: ";
                for (const auto& instruction : instructions) {
                    std::cout << instruction << " ";
                }
                std::cout << std::endl;

                std::cout << "Sequence: ";
                for (const auto& seq : trajectories) {
                    std::cout << seq << " ";
                }
                std::cout << std::endl;
            }

            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    void send_request(int trajectory)
    {
        auto request = std::make_shared<oss_interfaces::srv::TrajSec::Request>();
        request->data = trajectory;

        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
        }

        auto response_received_callback = [this](rclcpp::Client<oss_interfaces::srv::TrajSec>::SharedFuture future) {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(node_->get_logger(), "Request was successful.");
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Request failed.");
            }
        };

        auto future_result = client_->async_send_request(request, response_received_callback);
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<oss_interfaces::srv::TrajSec>::SharedPtr client_;
};


class TrajFallback : public BT::SyncActionNode
{
public:
    TrajFallback(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

        client_ = node_->create_client<oss_interfaces::srv::StatusLog>("status_log");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        auto blackboard = config().blackboard;

        std::vector<std::string> instructions;
        std::vector<int> trajectories;

        blackboard->get("instructions", instructions);
        blackboard->get("trajectories", trajectories);

        send_status_log("Traj updated", true, 1);

        return BT::NodeStatus::FAILURE;
    }

private:
    void send_status_log(const std::string& traj_status, bool gripper_status, int32_t current_traj)
    {
        auto request = std::make_shared<oss_interfaces::srv::StatusLog::Request>();
        request->traj_status = traj_status;
        request->gripper_status = gripper_status;
        request->current_traj = current_traj;

        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
        }

        auto response_received_callback = [this](rclcpp::Client<oss_interfaces::srv::StatusLog>::SharedFuture future) {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(node_->get_logger(), "Status log request was successful.");
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Status log request failed.");
            }
        };

        auto future_result = client_->async_send_request(request, response_received_callback);
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<oss_interfaces::srv::StatusLog>::SharedPtr client_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("behavior_tree_node");

    rclcpp::Rate rate(10);

	BT::BehaviorTreeFactory factory;

	auto blackboard = BT::Blackboard::create();
	blackboard->set("node", node);

    factory.registerNodeType<GetTrajectory>("GetTrajectory");
    factory.registerNodeType<Pick>("Pick");
    factory.registerNodeType<Drop>("Drop");
    factory.registerNodeType<Traj>("Traj");
    factory.registerNodeType<TrajFallback>("TrajFallback");

	std::string pkgpath = ament_index_cpp::get_package_share_directory("oss_behavior_tree");
    std::string xml_file = pkgpath + "/xml/oss_arm_bt2.xml";
	
	auto tree = factory.createTreeFromFile(xml_file, blackboard);

	// Create a Groot2 publisher instance
	// BT::Groot2Publisher publisher(tree);

    while (rclcpp::ok()) 
    {
        tree.tickWhileRunning();
        rclcpp::spin_some(node);

        std::this_thread::sleep_for(std::chrono::milliseconds(3000));

        rate.sleep();
    }

  	rclcpp::shutdown();

	return 0;
}