#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_final_homework_interfaces/msg/turtle_array.hpp"

using namespace my_final_homework_interfaces;

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
    }

private:
    void poseControlCallBack(turtlesim::msg::Pose::SharedPtr pred_turtle_pose)
    {
        pred_turtle_pose_ = *pred_turtle_pose.get();
    }

    void aliveTurtlesCallback(my_final_homework_interfaces::msg::TurtleArray::SharedPtr msg)
    {
        current_alive_turtles_ = *msg.get();
        // if (current_alive_turtles_.turtles.size())
        // {
            auto target = current_alive_turtles_.turtles[0];

            lookToTurtle(&target);
        // }
    }

    void lookToTurtle(msg::Turtle *target)
    {
        double direction_x = target->turtle_position.x - pred_turtle_pose_.x;
        double direction_y = target->turtle_position.y - pred_turtle_pose_.y;
        double theta = std::atan2(direction_x, direction_y);

        auto my_twist = geometry_msgs::msg::Twist();
        my_twist.angular.x = theta - pred_turtle_pose_.theta;
        RCLCPP_INFO(get_logger(), "theta: %f target theta: %f", pred_turtle_pose_.theta, theta);
        cmd_vel_publisher_->publish(my_twist);
    }

    void moveTowardsTo()
    {

    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<my_final_homework_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_subscriber_;
    my_final_homework_interfaces::msg::TurtleArray current_alive_turtles_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    turtlesim::msg::Pose pred_turtle_pose_;
};

int main(int args, char **argv)
{
    rclcpp::init(args, argv);
    auto node = std::make_shared<TurtleControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}