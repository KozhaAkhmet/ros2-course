#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_final_homework_interfaces/msg/turtle_array.hpp"
#include "turtlesim/srv/kill.hpp"

using namespace my_final_homework_interfaces;

#define PI 3.14

class TurtleControlNode : public rclcpp::Node
{
public:
    TurtleControlNode() : Node("turtle_control")
    {
        RCLCPP_INFO(this->get_logger(), "Turtle control node is active.");

        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10,
                                                                           std::bind(&TurtleControlNode::poseControlCallBack, this, std::placeholders::_1));

        alive_turtles_subscriber_ = this->create_subscription<my_final_homework_interfaces::msg::TurtleArray>("alive_turtles", 10,
                                                                                                              std::bind(&TurtleControlNode::aliveTurtlesCallback, this, std::placeholders::_1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        update_loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TurtleControlNode::updateTimer, this));

        // target_turtle_.turtle_position.x = 10.0;
        // target_turtle_.turtle_position.y = 4.0;
    }

private:
    void poseControlCallBack(const turtlesim::msg::Pose::SharedPtr pred_turtle_pose)
    {
        this->pred_turtle_pose_ = *pred_turtle_pose.get();
        // RCLCPP_INFO(get_logger(), "from subscribe pred turtle x: %f y: %f", pred_turtle_pose_.x, pred_turtle_pose_.y );

    }

    void aliveTurtlesCallback(my_final_homework_interfaces::msg::TurtleArray::SharedPtr msg)
    {
        current_alive_turtles_ = *msg.get();
        RCLCPP_INFO(get_logger(), "size of the alive vector %d", current_alive_turtles_.turtles.size());
        target_turtle_ = current_alive_turtles_.turtles.back();
         RCLCPP_INFO(get_logger(), "target turtle x: %f y: %f", target_turtle_.turtle_position.x, target_turtle_.turtle_position.y );
 
        RCLCPP_INFO(get_logger(), "target turtle is %s", target_turtle_.turtle_name.c_str());
    }

    void lookAndMoveToTurtle()
    {
        
        double direction_x = target_turtle_.turtle_position.x - pred_turtle_pose_.x;
        double direction_y = target_turtle_.turtle_position.y - pred_turtle_pose_.y;
        distance_ = sqrt(direction_x * direction_x + direction_y * direction_y);
        auto my_twist = geometry_msgs::msg::Twist();

        if (distance_ > 0.5)
        {
            double theta = std::atan2(direction_y, direction_x);

            double diff = theta - pred_turtle_pose_.theta;

            if (diff > M_PI)
                diff -= 2*M_PI;
            else if (diff < -M_PI)
                diff += 2*M_PI;
            my_twist.angular.z = 6*diff;
            my_twist.angular.x = 0;
            my_twist.linear.x = 2*distance_;
            my_twist.linear.y = 0;
            // RCLCPP_INFO(get_logger(), "pred theta: %f target theta: %f difference: %f distance: %f", pred_turtle_pose_.theta, theta, (pred_turtle_pose_.theta - theta), distance_);
            RCLCPP_INFO(get_logger(),"target pos: x:%f y: %f | pred pos: x: %f y: %f",
                                                                                target_turtle_.turtle_position.x,
                                                                                target_turtle_.turtle_position.y,
                                                                                this->pred_turtle_pose_.x,
                                                                                this->pred_turtle_pose_.y);
        }
        else
        {
            my_twist.angular.z = 0;
            my_twist.angular.x = 0;
            my_twist.linear.x = 0;
            threads_.push_back(std::thread(std::bind(&TurtleControlNode::callKillTargetTurtle, this)));
        }
        cmd_vel_publisher_->publish(my_twist);
    }

    void callKillTargetTurtle()
    {
        auto client = this->create_client<turtlesim::srv::Kill>("kill");

        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for kill service to be up....");
        }

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = target_turtle_.turtle_name;

        auto future = client->async_send_request(request);
        RCLCPP_INFO(get_logger(), "target turtle %s has been killed. Next is %d", target_turtle_.turtle_name.c_str(), index_ + 3);
        
        killed_turtle_name = target_turtle_.turtle_name;
    }

    void updateTimer()
    {
        if (target_turtle_.turtle_name != killed_turtle_name)
        {
            lookAndMoveToTurtle();
        }
        // RCLCPP_INFO(get_logger(), "pred turtle x: %f y: %f", pred_turtle_pose_.x, pred_turtle_pose_.y );

    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<my_final_homework_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_subscriber_;
    my_final_homework_interfaces::msg::TurtleArray current_alive_turtles_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    my_final_homework_interfaces::msg::Turtle target_turtle_;
    std::string killed_turtle_name;
    double distance_;
    std::vector<std::thread> threads_;

    turtlesim::msg::Pose pred_turtle_pose_;
    rclcpp::TimerBase::SharedPtr update_loop_timer_;
};

int main(int args, char **argv)
{
    rclcpp::init(args, argv);
    auto node = std::make_shared<TurtleControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}