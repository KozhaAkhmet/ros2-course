#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberCount : public rclcpp::Node
{
public:
    NumberCount() : Node("number_count")
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10,
                                                                                std::bind(&NumberCount::callbackNumberCount, this, std::placeholders::_1));
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
    }

private:
    void callbackNumberCount(const example_interfaces::msg::Int64::SharedPtr number)
    {
        number->data = number->data + temp;
        temp = number->data;
        RCLCPP_INFO(this->get_logger(), "%d", number->data);
        publisher_ -> publish(number->data);
    }
    void publishNumberCount()
    {
        auto number_count = example_interfaces::msg::Int64();
        publisher_->publish(number_count);
    }
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    int64_t temp;
};

int main(int args, char **argv)
{
    rclcpp::init(args, argv);
    auto node = std::make_shared<NumberCount>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}