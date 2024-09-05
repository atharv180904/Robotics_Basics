#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class SinusoidalMovement : public rclcpp::Node
{
public:
    SinusoidalMovement()
    : Node("sinusoidal_movement")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SinusoidalMovement::timer_callback, this));
        
        // parameters
        frequency_ = 0.5;  
        amplitude_ = 0.5;  
        linear_speed_ = 0.2;  
        time_ = 0.0;
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::Twist msg;

        // angular velocity based on the sinusoidal equation
        msg.linear.x = linear_speed_;
        msg.angular.z = amplitude_ * std::sin(2.0 * M_PI * frequency_ * time_);

        // Publish 
        publisher_->publish(msg);

        // increment
        time_ += 0.1;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double frequency_;
    double amplitude_;
    double linear_speed_;
    double time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SinusoidalMovement>());
    rclcpp::shutdown();
    return 0;
}
