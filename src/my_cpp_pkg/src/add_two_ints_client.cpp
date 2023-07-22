#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClientNode : public rclcpp::Node
{
public:
    AddTwoIntsClientNode() : Node("add_two_ints_client")
    {
        
        // thread1_ = std::thread(std::bind(&AddTwoIntsClientNode::callAddtwoIntsService, this, 1, 4));
        
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddtwoIntsService, this, 1, 4)));
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddtwoIntsService, this, 4, 8)));
    }

    void callAddtwoIntsService(int a, int b)
    {
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for server to be up....");
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        auto future = client->async_send_request(request);

        try
        {
            auto responce = future.get();
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", a, b, responce->sum);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "service call failed");
        }
    }

private:
    // std::thread thread1_;
    std::vector<std::thread> threads_;  
};

int main(int args, char **argv)
{
    rclcpp::init(args, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}