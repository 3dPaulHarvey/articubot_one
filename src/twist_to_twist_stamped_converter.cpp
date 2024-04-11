#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistToTwistStampedConverter : public rclcpp::Node
{
public:
    TwistToTwistStampedConverter() : Node("twist_to_twist_stamped_converter")
    {
        // Initialize publisher and subscriber
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/diffbot_base_controller/cmd_vel", 10);
        
        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                auto twist_stamped_msg = geometry_msgs::msg::TwistStamped();
                twist_stamped_msg.header.stamp = this->get_clock()->now();
                twist_stamped_msg.twist = *msg;
                publisher_->publish(twist_stamped_msg);
            });
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistToTwistStampedConverter>());
    rclcpp::shutdown();
    return 0;
}

