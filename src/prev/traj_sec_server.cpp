#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "oss_interfaces/srv/traj_sec.hpp"

using namespace std::chrono_literals;

class TrajSecServer : public rclcpp::Node
{
public:
    TrajSecServer()
        : Node("traj_sec_server")
    {
        service_ = this->create_service<oss_interfaces::srv::TrajSec>(
            "/trajectory_sequence",
            std::bind(&TrajSecServer::handle_service, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "TrajSec service server ready.");
    }

private:
    void handle_service(
        const std::shared_ptr<oss_interfaces::srv::TrajSec::Request> request,
        std::shared_ptr<oss_interfaces::srv::TrajSec::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received request");
        std::this_thread::sleep_for(std::chrono::seconds(5));
        RCLCPP_INFO(this->get_logger(), "Trajectory: %d", request->data);
        
        response->success = true;
    }

    rclcpp::Service<oss_interfaces::srv::TrajSec>::SharedPtr service_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TrajSecServer>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
