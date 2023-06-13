# Introduction to ROS: Controlling Turtlesim

In this tutorial, we will learn how to run the turtlesim simulator and control it using both teleop and Python. 

## Table of Contents

1. [Installing ROS and Turtlesim](#installing-ros-and-turtlesim)
2. [Running Turtlesim](#running-turtlesim)
3. [Controlling Turtlesim using Teleop](#controlling-turtlesim-using-teleop)
4. [Controlling Turtlesim using Python](#controlling-turtlesim-using-python)

## 1. Installing ROS and Turtlesim <a id="installing-ros-and-turtlesim"></a>

First, let's install ROS and the turtlesim package if they are not already installed.

```bash
pip install rospkg      # Install ROS Python package
sudo apt-get install ros-noetic-ros-tutorials    # Install ROS tutorials package
```

## 2. Running Turtlesim <a id="running-turtlesim"></a>

To run the turtlesim simulator, open a new terminal window and run the following command:

```bash
rosrun turtlesim turtlesim_node
```
- you first need to run the ROS master node using the `roscore` command in a new terminal window.

This will start the turtlesim simulator, and you should see a new window with a turtle and a rectangular grid.

![Turtlesim](../../images/turtlesim_node.png)

## 3. Controlling Turtlesim using Teleop <a id="controlling-turtlesim-using-teleop"></a>

Now, let's control the turtle in the turtlesim simulator using the teleop package.

In a new terminal window, run the following command to start the teleop node:

```bash
!rosrun turtlesim turtle_teleop_key
```

You can now control the turtle using the arrow keys on your keyboard. Experiment with different key combinations to move the turtle in different directions.

![Turtlesim](../../images/turtlesim_teleop.png)

## 4. Controlling Turtlesim using Python <a id="controlling-turtlesim-using-python"></a>

Next, let's control the turtle in the turtlesim simulator using Python. We will use the `rospy` package to publish velocity commands to control the turtle's movement.

First, import the necessary packages in a Python code cell:

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
```

Next, initialize the ROS node and create a `Twist` message object to store the velocity commands:

```python
rospy.init_node('turtle_controller')
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)  # 10 Hz
```

Now, you can define a function to send velocity commands to control the turtle:

```python
def move_turtle(linear_vel, angular_vel):
    twist = Twist()
    twist.linear.x = linear_vel
    twist.angular.z = angular_vel
    pub.publish(twist)
```

You can call the `move_turtle` function with different linear and angular velocities to control the turtle. For example:

```python
move_turtle(1.0, 0.0)  # Move forward with linear velocity of 1.0
rospy.sleep(2.0)  # Sleep for 2 seconds
move_turtle(0.0, 

1.0)  # Rotate clockwise with angular velocity of 1.0
rospy.sleep(2.0)  # Sleep for 2 seconds
move_turtle(0.0, 0.0)  # Stop the turtle
```

Experiment with different velocity values and movement patterns to control the turtle in the turtlesim simulator.

## Conclusion

In this tutorial, we learned how to run the turtlesim simulator and control it using both teleop and Python. Turtlesim provides a simple environment for learning and experimenting with ROS concepts.

Feel free to explore more features of turtlesim, ROS, and the various packages available to enhance your robotics projects.

Happy ROS coding!