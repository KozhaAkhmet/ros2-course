#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_states.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

class PanelNode : public rclcpp::Node
{
public:
    PanelNode() : Node("panel_node")
    {
        publisher_ = this->create_publisher<my_robot_interfaces::msg::LedStates>("led_panel_state",10);

        timer_ = this->create_wall_timer(std::chrono::seconds(4),std::bind(&PanelNode::publishLedStates,this));

        server_ = this->create_service<my_robot_interfaces::srv::SetLed>("set_led",
                                                                            std::bind(&PanelNode::callbackSetLed, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Set led service server has benn started");
    }   

private:
    void callbackSetLed(const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
                            const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
    {
        try
        {
            response->success = true;
            if(request->state)
                led_panel[request->led_number - 1] = 1;
            else if(!request->state)
                led_panel[request->led_number - 1] = 0;
            else 
                response->success = false;
        }
        catch (const std::exception &e)
        {
            response->success = false;
        }

        RCLCPP_INFO(this->get_logger(), "Led number: %d  State: %d | Success: %d   led panel: %d %d %d", request->led_number, request->state, response->success,
                                                                                led_panel[0], led_panel[1], led_panel[2]);
    }
    void publishLedStates()
    {
        auto msg = my_robot_interfaces::msg::LedStates();
        msg.led_state = led_panel;
        publisher_->publish(msg);
    }
    rclcpp::Publisher<my_robot_interfaces::msg::LedStates>::SharedPtr publisher_;

    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr server_;
    std::vector<int64_t> led_panel = std::vector<int64_t>(3);
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int args, char **argv)
{
    rclcpp::init(args, argv);
    auto node = std::make_shared<PanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}