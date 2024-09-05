#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>

class KeyboardControl : public rclcpp::Node
{
public:
    KeyboardControl()
    : Node("keyboard_control"), last_key_(' ')
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), // Reduced timer interval for higher reactivity
            [this]() { this->timer_callback(); });
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::Twist msg;
        char c = get_key();
        
        // if key is pressed, process it
        if (c != '\0') {
            last_key_ = c;
        } else {
            // no key is pressed, stop the robot
            last_key_ = ' ';
        }

        // velocity based on the last pressed key
        switch (last_key_) {
            case 'w':  // forward
                msg.linear.x = 0.5;
                msg.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Robot is moving forward");
                break;
            case 's':  // backward
                msg.linear.x = -0.5;
                msg.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Robot is moving backward");
                break;
            case 'a':  // left
                msg.linear.x = 0.0;
                msg.angular.z = 0.5;
                RCLCPP_INFO(this->get_logger(), "Robot is turning left");
                break;
            case 'd':  // right
                msg.linear.x = 0.0;
                msg.angular.z = -0.5;
                RCLCPP_INFO(this->get_logger(), "Robot is turning right");
                break;
            case ' ':  // stop
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Robot is stopped");
                break;
            default:
                // space->stop
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Robot is stopped");
                break;
        }

        publisher_->publish(msg);
    }

    char get_key()
    {
        struct termios oldt, newt;
        char ch = '\0';
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        if (read(STDIN_FILENO, &ch, 1) == 1) {
            
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    char last_key_; // last pressed key
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    //create instance of class
    rclcpp::spin(std::make_shared<KeyboardControl>());
    rclcpp::shutdown();
    return 0;
}
