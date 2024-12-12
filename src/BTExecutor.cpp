#include "ras_bt_framework/BtExecutor.hpp"

namespace ras_bt_framework
{
    BTExecutor::BTExecutor(std::shared_ptr<BT::BehaviorTreeFactory> bt_factory, const rclcpp::NodeOptions & options) :
     rclcpp::Node("bt_executor", options),bt_factory_(bt_factory)
    {
        this->action_server_ = rclcpp_action::create_server<BTExecutor::BTInterface>(
            this,
            "bt_executor",
            std::bind(&BTExecutor::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&BTExecutor::handle_cancel, this, std::placeholders::_1),
            std::bind(&BTExecutor::handle_accepted, this, std::placeholders::_1));
    }

    rclcpp_action::GoalResponse BTExecutor::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const BTExecutor::BTInterface::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request with id %d", goal->bt_path);
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse BTExecutor::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void BTExecutor::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Goal accepted");
        std::thread{std::bind(&BTExecutor::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void BTExecutor::execute(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        auto feedback = std::make_shared<BTExecutor::BTInterface::Feedback>();
        auto result = std::make_shared<BTExecutor::BTInterface::Result>();
        
        const std::string& bt_path = goal_handle->get_goal()->bt_path;
        bt_factory_->clearRegisteredBehaviorTrees();
        bt_factory_->registerBehaviorTreeFromFile(bt_path);
        BT::Tree tree = bt_factory_->createTree("MainTree");
        auto _delay = std::chrono::milliseconds(10);
        double rate = 1.0l/std::chrono::duration_cast<std::chrono::seconds>(_delay).count();
        rclcpp::Rate loop_rate(rate);
        BT::NodeStatus status = BT::NodeStatus::IDLE;
        while (rclcpp::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
            if(goal_handle->is_canceling()){
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            goal_handle->publish_feedback(feedback);
            feedback->status = std::to_string(static_cast<uint32_t>(status));
            status = tree.tickOnce();
            if(status == BT::NodeStatus::FAILURE) {
                
            }
            loop_rate.sleep();
        }
        if((rclcpp::ok()) && (status == BT::NodeStatus::SUCCESS)) {
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        } else {
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
        }
    }
}