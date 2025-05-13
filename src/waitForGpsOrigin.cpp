#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class WaitForGPSOrigin : public rclcpp::Node
{
public:
    WaitForGPSOrigin() : Node("lio_sam_waitForGpsOrigin")
    {
        client_ = this->create_client<std_srvs::srv::Trigger>("/lio_sam/gps/origin/initSuccess");

        RCLCPP_INFO(this->get_logger(), "Waiting for /lio_sam/gps/origin/initSuccess service...");

        // Wait for the service to be available
        while (!client_->wait_for_service(1s) && rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Service not available yet...");
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client_->async_send_request(request);

        auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

        if (result == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Service responded: %s", response->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Service responded with failure: %s", response->message.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service.");
        }
    }

private:
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaitForGPSOrigin>();
    // No need to spin — the constructor blocks until done
    rclcpp::shutdown();
    return 0;
}
