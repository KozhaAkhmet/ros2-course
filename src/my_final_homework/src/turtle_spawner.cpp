#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/kill.hpp"
#include "my_final_homework_interfaces/msg/turtle_array.hpp"

class TurtleSpawnerNode : public rclcpp::Node
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner")
    {
        RCLCPP_INFO(this->get_logger(), "Turtle spawner node is active.");

        this->declare_parameter("spawn_milliseconds", 1000);

        milliseconds_ = this->get_parameter("spawn_milliseconds").as_int();
        alive_turtles_publisher_ = this->create_publisher<my_final_homework_interfaces::msg::TurtleArray>("alive_turtles", 10);

        killed_turtle_subscription = this->create_subscription<my_final_homework_interfaces::msg::Turtle>("killed_turtle", 10,
                                                                                                              std::bind(&TurtleSpawnerNode::removeFromAliveTurltes, this, std::placeholders::_1));

        spawn_timer_ = this->create_wall_timer(std::chrono::milliseconds(milliseconds_),
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
                RCLCPP_INFO(this->get_logger(), "Spawning %s...", new_turlte_name_.c_str());

                my_final_homework_interfaces::msg::Turtle new_turtle;
                new_turtle.turtle_name = new_turlte_name_;
                new_turtle.turtle_position.x = x;
                new_turtle.turtle_position.y = y;
                // RCLCPP_INFO(get_logger(), "new turtle x: %f y: %f", new_turtle.turtle_position.x, new_turtle.turtle_position.y );
                
                try
                {
                   alive_turtles_.turtles.push_back(new_turtle);
                   alive_turtles_publisher_->publish(alive_turtles_);
                }
                catch(const std::exception& e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Alive turtle pushback failed.");
                }

                RCLCPP_INFO(this->get_logger(), "Publishing alive turtles....");
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Spawn service call failed");
            }
 
    }

    void spawnRandomTurtles()
    {
        threads_.push_back(std::thread(std::bind(&TurtleSpawnerNode::callSpawnTurtle, this,
                                                 std::rand() % 11,
                                                 std::rand() % 11,
                                                 (std::rand() % 10) / M_PI)));
    }

    void callKillTargetTurtle()
    {
        auto client = this->create_client<turtlesim::srv::Kill>("kill");

        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for kill service to be up....");
        }

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = killed_turtle.turtle_name;
        auto future = client->async_send_request(request);
        
        future.wait();

        RCLCPP_INFO(get_logger(), "target turtle %s has been killed.", killed_turtle.turtle_name.c_str());
        // Reminder do not forget to focus on the sequence of execution request and publishing, kill and remove killed turtle
        // TODO Replace Publisher to client so the spawner will remove trutle throgh service
    }

    void removeFromAliveTurltes(const my_final_homework_interfaces::msg::Turtle::SharedPtr msg)
    {
        killed_turtle = *msg.get();
        threads_.push_back(std::thread(std::bind(&TurtleSpawnerNode::callKillTargetTurtle, this)));
        

            for(auto i = alive_turtles_.turtles.begin(); i != alive_turtles_.turtles.end(); i++)
            {
                if(*i == *msg.get())
                {   
                    try
                    {
                        alive_turtles_.turtles.erase(i);
                        alive_turtles_publisher_->publish(alive_turtles_);
                        RCLCPP_WARN(this->get_logger(), "%s turtle is erased!.", i->turtle_name.c_str());
                    }
                    catch(const std::exception& e)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Erasing the turtle is failed.");
                    }
                    
                }
            }
        
    }

    rclcpp::Publisher<int64_t>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr spawn_timer_;
    std::vector<std::thread> threads_;
    my_final_homework_interfaces::msg::TurtleArray alive_turtles_;
    rclcpp::Publisher<my_final_homework_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_publisher_;
    rclcpp::Subscription<my_final_homework_interfaces::msg::Turtle>::SharedPtr killed_turtle_subscription;
    std::string new_turlte_name_;
    my_final_homework_interfaces::msg::Turtle killed_turtle;
    int milliseconds_;
};

int main(int args, char **argv)
{
    rclcpp::init(args, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}