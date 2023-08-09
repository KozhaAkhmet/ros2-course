#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery_node"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Battery Node now is active!");

        // timer_ = this->create_wall_timer(std::chrono::seconds(4),
        //                                  std::bind(&BatteryNode::callSetLedService, this, 2, "on"));
        threads_.push_back(std::thread(std::bind(&BatteryNode::callSetLedService, this, 2, "on")));
    }

private:
    void callSetLedService(int a, bool b)
    {
        auto client = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");

        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for set_led service....");
        }

        auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
        request->led_number = a;
        request->state = b;

        auto future = client->async_send_request(request);

        try
        {
            auto responce = future.get();
            RCLCPP_INFO(this->get_logger(), "%d + %s = %d", a, b, responce->success);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::thread> threads_;  
    int counter_;
};

int main(int args, char **argv)
{
    rclcpp::init(args, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}