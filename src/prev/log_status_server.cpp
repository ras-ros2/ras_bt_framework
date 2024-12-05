#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "oss_interfaces/srv/status_log.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class StatusLogServer : public rclcpp::Node
{
public:
    StatusLogServer() : Node("status_log_server")
    {
        service_ = this->create_service<oss_interfaces::srv::StatusLog>(
            "status_log", std::bind(&StatusLogServer::handle_service, this, _1, _2));
    }

private:
    void handle_service(const std::shared_ptr<oss_interfaces::srv::StatusLog::Request> request,
                        std::shared_ptr<oss_interfaces::srv::StatusLog::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received request:\nTraj Status: %s\nGripper Status: %s\nCurrent Traj: %d",
                    request->traj_status.c_str(), request->gripper_status ? "true" : "false", request->current_traj);

        std::this_thread::sleep_for(std::chrono::seconds(5));

        // Respond with success
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Response: success = true");
    }

    rclcpp::Service<oss_interfaces::srv::StatusLog>::SharedPtr service_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StatusLogServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
