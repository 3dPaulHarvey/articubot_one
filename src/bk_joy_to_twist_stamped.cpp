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
        // Example conversion logic: map axes directly to linear and angular velocities
        auto twist_stamped_msg = geometry_msgs::msg::TwistStamped();
        twist_stamped_msg.header.stamp = this->get_clock()->now();
        
        // Assuming axis 1 controls linear velocity and axis 0 controls angular velocity
        twist_stamped_msg.twist.linear.x = msg->axes[1]; // Adjust index and scale as necessary
        twist_stamped_msg.twist.angular.z = msg->axes[0]; // Adjust index and scale as necessary

        publisher_->publish(twist_stamped_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToTwistStamped>());
    rclcpp::shutdown();
    return 0;
}
