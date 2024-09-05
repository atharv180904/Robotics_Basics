# robotics_basics
Robotics Assignment

Screen Recordings: https://drive.google.com/drive/folders/1B32L612AawVTa-RNxmTROJWoLo_8OWUm?usp=drive_link

# KeyboardControl Package Documentation (Question 1)

This ROS 2 node allows you to control a robot using your keyboard. The node listens for key presses and translates them into movement commands, which are published to the `/cmd_vel` topic.

## Key Features
- **Move forward (W):** Sends a forward velocity command.
- **Move backward (S):** Sends a reverse velocity command.
- **Turn left (A):** Sends a command to rotate the robot counterclockwise.
- **Turn right (D):** Sends a command to rotate the robot clockwise.
- **Stop (Spacebar):** Stops all motion.

## Code Breakdown

- **Publisher:** The node publishes `geometry_msgs::msg::Twist` messages to `/cmd_vel`, which controls the robot's linear and angular velocity.
- **Timer:** The node uses a timer to check for keyboard input every 10 milliseconds. If a key is pressed, it triggers the appropriate movement command.
- **Key Handling:** It uses terminal settings to capture key presses without waiting for the "Enter" key. The `get_key()` function captures a single character from the terminal and returns it for processing.
- **Movement Logic:** Based on the last key pressed, the robot moves in the corresponding direction or stops if no key is pressed.

## Running the Node

To run the node, git-clone the package & compile it in your ROS 2 workspace and execute the following:

```bash
ros2 run question_1_pkg keyboard_control
```

You should see feedback in the terminal, indicating the robot's actions based on your key presses.

### Dependencies
- `rclcpp`: Core ROS 2 C++ client library.
- `geometry_msgs/msg/twist`: Message type used to send velocity commands to the robot.

