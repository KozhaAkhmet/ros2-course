#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery_node"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Battery Node now is active!");

        battery_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&BatteryNode::batteryStatusUpdate, this));
        
    }

private:
    void callSetLedService(int64_t a, bool b)
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
        future.wait();
        callbackCallSetLed(future.get(), a, b);
        
    }
    void callbackCallSetLed(const my_robot_interfaces::srv::SetLed::Response::SharedPtr responce, int64_t led_number, bool states)
    {
        try
        {
            RCLCPP_INFO(this->get_logger(), "Sending a request....\n led_number: %d | state: %d | Success: %d", 
                                                                                    led_number, states, responce->success);
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    void batteryStatusUpdate()
    {
        // threads_.push_back(std::thread(std::bind(&BatteryNode::callSetLedService, this, 2, "on")));
        auto time_now = get_current_time_seconds();
        if (battery_ == "full")
        {
            if (time_now - last_time_ > 4.0)
            {
                battery_ = "empty";
                RCLCPP_INFO(this->get_logger(),"Battery is empty.");
                last_time_ = time_now;
                threads_.push_back(std::thread(std::bind(&BatteryNode::callSetLedService, this, 3, true)));
            }
        }
        else if( time_now - last_time_ > 6.0)
            {
                battery_ = "full";
                RCLCPP_INFO(this->get_logger(),"Battery is charged.");
                last_time_ = time_now;
                threads_.push_back(std::thread(std::bind(&BatteryNode::callSetLedService, this, 3, false)));
            }
    }

    double get_current_time_seconds()
    {
        return this->get_clock()->now().seconds();
    }

    rclcpp::TimerBase::SharedPtr battery_timer_;
    double last_time_ = BatteryNode::get_current_time_seconds();
    std::vector<std::thread> threads_;  
    std::string battery_ = "full";
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