#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberPublisher: public rclcpp::Node
{
public:
    NumberPublisher(): Node("number_publisher")
    {
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&NumberPublisher::publishNumber, this));
    }
private:
    void publishNumber()
    {
        auto number = example_interfaces::msg::Int64();
        number.data = 1;
        publisher_->publish(number);
    }
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int args, char **argv)
{
    rclcpp::init(args,argv);
    auto node = std::make_shared<NumberPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}