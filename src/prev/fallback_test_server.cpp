#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

class FallbackService : public rclcpp::Node
{
public:
    FallbackService()
        : Node("fallback_service")
    {
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_bool_service", std::bind(&FallbackService::handle_service, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Fallback server ready.");
    }

private:
    void handle_service(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        // Log the received boolean value
        RCLCPP_INFO(this->get_logger(), "Received SetBool request: %s", request->data ? "true" : "false");

        response->success = true;
        response->message = "SetBool request handled successfully.";

        RCLCPP_INFO(this->get_logger(), "Response: success = true, message = '%s'", response->message.c_str());
    }

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FallbackService>());
    rclcpp::shutdown();
    return 0;
}
