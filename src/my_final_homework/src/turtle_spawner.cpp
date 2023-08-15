#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/msg/pose.hpp"
#include "my_final_homework_interfaces/msg/turtle_array.hpp"

class TurtleSpawnerNode : public rclcpp::Node
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner")
    {
        RCLCPP_INFO(this->get_logger(), "Turtle spawner node is active.");

        alive_turtles_publisher_ = this->create_publisher<my_final_homework_interfaces::msg::TurtleArray>("alive_turtles", 10);

        spawn_timer_ = this->create_wall_timer(std::chrono::seconds(3),
                                               std::bind(&TurtleSpawnerNode::spawnRandomTurtles, this));
        
        alive_turtles_.turtles.clear();
    }

private:
    void callSpawnTurtle(float x, float y, float theta)
    {
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");

        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for /spawn to be up....");
        }

        if (alive_turtles_.turtles.size() < 5)
        {
            auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
            request->x = x;
            request->y = y;
            request->theta = theta;
            auto future = client->async_send_request(request);
            future.wait();
            try
            {
                auto responce = future.get();
                new_turlte_name_ = responce->name;
                // alive_turtles_.turtles
                RCLCPP_INFO(this->get_logger(), "Spawning %s...", new_turlte_name_.c_str());
                // Subscribe to spawned turtle by its name like (turlte name)/Pose

                std::string subscription = new_turlte_name_ + "/pose";
                RCLCPP_INFO(this->get_logger(), "Subscribing to %s...",subscription.c_str());
                new_turtle_pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(subscription.c_str(),
                                                                                            10,
                                                                                            std::bind(&TurtleSpawnerNode::callbackGetNewTurtlePose, this, std::placeholders::_1)
                                                                                            );

            my_final_homework_interfaces::msg::Turtle new_turtle;
            new_turtle.turtle_name = new_turlte_name_;
            new_turtle.turtle_position = new_turtle_pose_;
            alive_turtles_.turtles.push_back(new_turtle);
            RCLCPP_INFO(this->get_logger(), "Subscribed to new turtle pose. Publishing alive turtles....");
            alive_turtles_publisher_->publish(alive_turtles_);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Spawn service call failed");
            }
        }
    }

    void spawnRandomTurtles()
    {
        auto random_x = std::rand() / 10;
        threads_.push_back(std::thread(std::bind(&TurtleSpawnerNode::callSpawnTurtle, this,
                                                 std::rand() % 10,
                                                 std::rand() % 10,
                                                 (std::rand() % 10) / 10)));
    }

    void updateAliveTurtles()
    {
    }
    void callbackGetNewTurtlePose(const turtlesim::msg::Pose::SharedPtr msg)
    {
        new_turtle_pose_ = *msg.get();
    }

    rclcpp::Publisher<int64_t>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr spawn_timer_;
    std::vector<std::thread> threads_;
    my_final_homework_interfaces::msg::TurtleArray alive_turtles_;
    rclcpp::Publisher<my_final_homework_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr new_turtle_pose_subscriber_;
    turtlesim::msg::Pose new_turtle_pose_;
    std::string new_turlte_name_;

    int counter_;
};

int main(int args, char **argv)
{
    rclcpp::init(args, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}