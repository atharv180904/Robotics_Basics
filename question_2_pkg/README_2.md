# SinusoidalMovement Node Documentation

This ROS 2 node makes a robot move in a sinusoidal pattern. The robot moves forward with a constant linear velocity, while its angular velocity varies in a sinusoidal pattern.

## Key Features
- **Linear movement:** The robot moves forward at a constant speed.
- **Sinusoidal turning:** The robot's turning speed follows a sine wave, creating a smooth oscillating motion.

## Code Breakdown

- **Publisher:** The node publishes `geometry_msgs::msg::Twist` messages to the `/cmd_vel` topic, which controls the robot's movement.
- **Timer:** A timer is used to periodically update and publish the robot's velocity commands. It triggers every 100 milliseconds.
- **Sinusoidal Movement:** The angular velocity is calculated using the sinusoidal equation: `amplitude * sin(2π * frequency * time)`. This gives the robot its oscillating turning behavior while maintaining a steady forward speed.

## Parameters

- `frequency_`: The frequency of the sinusoidal wave controlling the robot's angular velocity (in Hz).
- `amplitude_`: The maximum turning speed of the robot.
- `linear_speed_`: The constant forward speed of the robot.
- `time_`: Tracks the progression of time to calculate the sinusoidal wave.

## Running the Node

To run the executable, git-clone the package, compile it in your ROS 2 workspace and execute the following:

```bash
ros2 run question_2_pkg sinusoidal_movement
```

The robot will move forward in a wavy pattern, alternating between turning left and right according to the sine wave.

### Dependencies
- `rclcpp`: ROS 2 C++ client library.
- `geometry_msgs/msg/twist`: Used to publish the velocity commands.
