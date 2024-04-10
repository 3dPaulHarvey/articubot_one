#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoyListener : public rclcpp::Node
{
public:
    JoyListener() : Node("joy_listener")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyListener::joy_callback, this, std::placeholders::_1));
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        std::string buttons_str = "Buttons:";
        for (size_t i = 0; i < msg->buttons.size(); ++i) {
            buttons_str += " " + std::to_string(msg->buttons[i]);
        }

        std::string axes_str = "Axes:";
        for (size_t i = 0; i < msg->axes.size(); ++i) {
            axes_str += " " + std::to_string(msg->axes[i]);
        }

        RCLCPP_INFO(this->get_logger(), "%s %s", buttons_str.c_str(), axes_str.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyListener>());
    rclcpp::shutdown();
    return 0;
}
