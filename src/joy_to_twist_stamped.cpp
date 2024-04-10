#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class JoyToTwistStamped : public rclcpp::Node
{
public:
    JoyToTwistStamped() : Node("joy_to_twist_stamped")
    {
        // Initialize publisher and subscriber
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/diffbot_base_controller/cmd_vel", 10);
        
        subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyToTwistStamped::joy_callback, this, std::placeholders::_1));
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto twist_stamped_msg = geometry_msgs::msg::TwistStamped();
        twist_stamped_msg.header.stamp = this->get_clock()->now();
        
        // Linear velocity (forward/backward) controlled by left stick vertical axis
        // Assuming axis 1 is the vertical axis of the left stick
        // Adjust the index and scale factor as necessary for your controller
        twist_stamped_msg.twist.linear.x = msg->axes[1] * linear_scale;

        // Angular velocity (turning) controlled by right stick horizontal axis
        // Assuming axis 3 is the horizontal axis of the right stick
        // Adjust the index and scale factor as necessary for your controller
        twist_stamped_msg.twist.angular.z = msg->axes[0] * angular_scale;

        publisher_->publish(twist_stamped_msg);
    }


    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
    double linear_scale = 0.5; 
    double angular_scale = 0.5; 
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToTwistStamped>());
    rclcpp::shutdown();
    return 0;
}
