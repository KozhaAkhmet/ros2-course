#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "my_final_homework_interfaces/msg/turtle_array.hpp"

class TurtleSpawnerNode : public rclcpp::Node
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner")
    {
        RCLCPP_INFO(this->get_logger(), "Turtle spawner node is active.");

        spawn_timer_ = this->create_wall_timer(std::chrono::seconds(3),
                                         std::bind(&TurtleSpawnerNode::spawnRandomTurtles, this));
        
    }

private:
    void callSpawnTurtle(float x, float y, float theta)
    {
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");

        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for /spawn to be up....");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        auto future = client->async_send_request(request);
        future.wait();
        try
        {
            auto responce = future.get();
            alive_turtles_.turtles
            RCLCPP_INFO(this->get_logger(),"Spawning %s...", responce->name.c_str());
            // Subscribe to spawned turtle by its name like (turlte name)/Pose
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Spawn service call failed");
        }
    }

    void spawnRandomTurtles()
    {
        auto random_x = std::rand() / 10;
        threads_.push_back(std::thread(std::bind(&TurtleSpawnerNode::callSpawnTurtle, this, 
        std::rand() % 10,
        std::rand() % 10,
        (std::rand() % 10)/ 10)));
    }

    void updateAliveTurtles()
    {

    }

    rclcpp::Publisher<int64_t>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr spawn_timer_;
    std::vector<std::thread> threads_;

    my_final_homework_interfaces::msg::TurtleArray alive_turtles_;
    
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