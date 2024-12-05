#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

class GripperService : public rclcpp::Node
{
public:
    GripperService()
        : Node("gripper_service")
    {
        gripper_service_ = this->create_service<std_srvs::srv::SetBool>(
            "gripper_control", std::bind(&GripperService::gripper_callback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Gripper Service is ready.");
    }

private:
    void gripper_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            set_vacuum_gripper(true);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            set_vacuum_gripper(false);
        }

        // std::this_thread::sleep_for(std::chrono::seconds(5)); // 5-second delay

        response->success = true;
    }

    void set_vacuum_gripper(bool state)
    {
        // Implement the logic to control the vacuum gripper here
        if (state)
        {
            RCLCPP_INFO(this->get_logger(), "Vacuum gripper is turned ON.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Vacuum gripper is turned OFF.");
        }
    }

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr gripper_service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
