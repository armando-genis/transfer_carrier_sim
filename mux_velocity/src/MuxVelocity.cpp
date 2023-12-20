// Ros2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"

// C++
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

class MuxVelocity : public rclcpp::Node
{
private:
    /* data */
    void warning_callback(const std_msgs::msg::Int32::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

public:
    MuxVelocity(/* args */);
    ~MuxVelocity();
};

MuxVelocity::MuxVelocity(/* args */) : Node("mux_velocity_node")
{
    subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        "warning_status", 10, std::bind(&MuxVelocity::warning_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "mux_velocity_node initialized");
}

MuxVelocity::~MuxVelocity()
{
}

void MuxVelocity::warning_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    auto velocity_cmd = geometry_msgs::msg::Twist();

    switch (msg->data)
    {
    case 1: // Stop immediately
        velocity_cmd.linear.x = 0.0;
        velocity_cmd.angular.z = 0.0;
        break;
    case 2: // Reduce velocity
        velocity_cmd.linear.x = 0.5;
        velocity_cmd.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "case 2");
        break;
    case 3: // Reduce velocity by 70%
        velocity_cmd.linear.x = 0.75;
        velocity_cmd.angular.z = 0.0;
        break;
    default: // Normal velocity
        // Set to normal values as per your application
        velocity_cmd.linear.x = 1.0;
        velocity_cmd.angular.z = 0.0;
        break;
    }

    publisher_->publish(velocity_cmd);
    RCLCPP_INFO(this->get_logger(), "Setting velocity: linear.x = '%f', angular.z = '%f'",
                velocity_cmd.linear.x, velocity_cmd.angular.z);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MuxVelocity>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}