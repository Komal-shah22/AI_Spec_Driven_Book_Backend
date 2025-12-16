# Chapter 3: Your First ROS 2 Application: Building a Simple Robotic System

## Purpose:
To guide beginners through the practical workflow of creating, building, and running a basic ROS 2 application.

## Learning Objectives:
- Set up a ROS 2 workspace and create new packages.
- Write simple publisher and subscriber nodes in Python.
- Build and run a ROS 2 application using `colcon`.
- Utilize basic ROS 2 command-line tools for introspection.
- Understand the concept of launch files for orchestrating multiple nodes.

## Expected Outputs:
- A functional ROS 2 workspace with custom publisher/subscriber nodes.
- Familiarity with `colcon` build system.
- Ability to use `ros2 run`, `ros2 node list`, `ros2 topic list`, `ros2 launch`.
- Working launch file for the simple application.

## Content Outline:
### 3.1 ROS 2 Workspaces: Organizing Your Code

In ROS 2, a **workspace** is a directory where you organize your source code, build your packages, and install your executables. It's a fundamental concept for managing your ROS 2 projects. A typical workspace structure looks like this:

```
<workspace_name>/
├── src/                # Contains your ROS 2 packages
│   ├── my_package_1/
│   ├── my_package_2/
│   └── ...
├── install/            # Where compiled executables and libraries are installed
├── log/                # Build and test logs
├── build/              # Intermediate build files
└── .colcon_bash        # Environment setup file
```

To create a workspace:
1.  Create the `src` directory: `mkdir -p <workspace_name>/src`
2.  Navigate into the workspace: `cd <workspace_name>`
3.  Source your ROS 2 installation (e.g., `source /opt/ros/humble/setup.bash`)
4.  Optionally, build an empty workspace to generate `install/` and `build/` directories: `colcon build`

### 3.2 Creating Your First ROS 2 Package (Python)

A **package** is the atomic unit of software in ROS 2. It contains nodes, launch files, message definitions, and other resources. We will create a Python package for our first application.

To create a new Python package within your workspace's `src` directory:

```bash
cd <workspace_name>/src
ros2 pkg create --build-type ament_python my_robot_app
```

This command creates a directory named `my_robot_app` with a basic structure, including a `setup.py` and `package.xml` file.

**`my_robot_app/package.xml`**: Contains metadata about your package (name, description, version, dependencies).
**`my_robot_app/setup.py`**: Defines how your Python code is built and installed. You'll specify your executable scripts here.
**`my_robot_app/my_robot_app/__init__.py`**: An empty file that makes the directory a Python package.
**`my_robot_app/my_robot_app/<your_node_file.py>`**: This is where you'll write your Python nodes.
### 3.3 Writing a Simple ROS 2 Publisher Node (Python)

A publisher node is responsible for sending data (messages) on a specific topic. Here's a simple Python publisher node that sends "Hello ROS 2 World" messages:

```python
# src/examples/ch3_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Import the standard String message type

class MinimalPublisher(Node):
    def __init__(self):
        # Initialize the Node with the name 'minimal_publisher'
        super().__init__('minimal_publisher')
        # Create a publisher that publishes String messages to the 'topic' topic
        # The second argument (10) is the QoS history depth
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        # Create a timer that calls the timer_callback function every 0.5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String() # Create a new String message object
        msg.data = 'Hello ROS 2 World: %d' % self.i # Set the message data
        self.publisher_.publish(msg) # Publish the message
        # Log the message to the console for debugging
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of our MinimalPublisher node
    minimal_publisher = MinimalPublisher()

    # Keep the node alive, allowing its callbacks (like the timer) to be called
    rclpy.spin(minimal_publisher)

    # Cleanly destroy the node and shutdown the ROS 2 Python client library
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation:**
-   `import rclpy` and `from rclpy.node import Node`: These lines import the necessary ROS 2 Python client library components.
-   `from std_msgs.msg import String`: This imports the `String` message type from the `std_msgs` package, which is a standard ROS 2 message for simple text.
-   `MinimalPublisher(Node)`: Our class inherits from `rclpy.node.Node`, making it a ROS 2 node.
-   `super().__init__('minimal_publisher')`: Initializes the base `Node` class and gives our node a unique name.
-   `self.create_publisher(String, 'topic', 10)`: Creates a publisher that will send `String` messages on a topic named `'topic'`. The `10` is the QoS history depth, indicating how many messages to buffer.
-   `self.create_timer(timer_period, self.timer_callback)`: Sets up a timer to call `timer_callback` every 0.5 seconds.
-   `timer_callback`: This function is called by the timer. It creates a `String` message, sets its `data` field, publishes it, and logs the action.
-   `rclpy.init(args=args)` and `rclpy.spin(minimal_publisher)`: These lines initialize the ROS 2 system and keep the node running, processing events and callbacks.
-   `minimal_publisher.destroy_node()` and `rclpy.shutdown()`: These ensure a clean shutdown of the node and the ROS 2 client library.
### 3.4 Writing a Simple ROS 2 Subscriber Node (Python)

A subscriber node is responsible for receiving data (messages) from a specific topic. Here's a simple Python subscriber node that listens for the "Hello ROS 2 World" messages:

```python
# src/examples/ch3_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Import the standard String message type

class MinimalSubscriber(Node):
    def __init__(self):
        # Initialize the Node with the name 'minimal_subscriber'
        super().__init__('minimal_subscriber')
        # Create a subscription that listens for String messages on the 'topic' topic
        # The callback function listener_callback will be called when a message is received
        self.subscription = self.create_subscription(
            String,       # Message type
            'topic',      # Topic name
            self.listener_callback, # Callback function
            10)           # QoS history depth
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        # Log the received message to the console
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of our MinimalSubscriber node
    minimal_subscriber = MinimalSubscriber()

    # Keep the node alive, allowing its callbacks (like the subscription) to be called
    rclpy.spin(minimal_subscriber)

    # Cleanly destroy the node and shutdown the ROS 2 Python client library
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation:**
-   `super().__init__('minimal_subscriber')`: Initializes the node with the name `minimal_subscriber`.
-   `self.create_subscription(String, 'topic', self.listener_callback, 10)`: Creates a subscriber that listens for `String` messages on the `'topic'` topic. When a message arrives, `listener_callback` is invoked.
-   `listener_callback(self, msg)`: This function receives the `String` message object and logs its content.
-   The `main` function structure is similar to the publisher, ensuring the node is properly initialized, spun, and shut down.
### 3.5 Building Your ROS 2 Package with `colcon`

Once you've created your ROS 2 package and added your Python nodes, the next step is to build it. ROS 2 uses `colcon` as its primary build tool. `colcon` is a versatile command-line tool that can compile, link, and install multiple packages within a workspace.

To build your `my_robot_app` package (and any other packages in your workspace):

1.  **Navigate to your workspace root:**
    ```bash
    cd <workspace_name>
    ```
2.  **Source your ROS 2 environment:**
    ```bash
    source /opt/ros/humble/setup.bash
    ```
3.  **Run `colcon build`:**
    ```bash
    colcon build
    ```
    This command will:
    -   Find all ROS 2 packages in your `src` directory.
    -   Compile your Python code (though Python packages mostly involve copying files).
    -   Install the necessary files (executables, Python modules) into the `install/` directory of your workspace.

After `colcon build` completes, you need to **source the workspace's setup file** to make your new package's executables and Python modules available in your environment.

```bash
source install/setup.bash
```

It's a common practice to add this `source install/setup.bash` command to your `~/.bashrc` file (after sourcing the main ROS 2 installation) so that your workspace is always sourced when you open a new terminal.

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

*(Note: Replace `~/ros2_ws` with the actual path to your workspace.)*
### 3.6 Running Your Nodes: `ros2 run` and Basic Introspection

After building your package, you can now run your nodes and observe their behavior using basic ROS 2 command-line tools.

#### `ros2 run`: Executing a Node

The `ros2 run` command allows you to execute a node from a compiled package. The general syntax is:

```bash
ros2 run <package_name> <executable_name>
```

For our `my_robot_app` package, if you've configured `setup.py` to make your publisher and subscriber nodes executable, you would run them like this (in separate terminals):

**Terminal 1 (Publisher):**
```bash
ros2 run my_robot_app minimal_publisher
```

**Terminal 2 (Subscriber):**
```bash
ros2 run my_robot_app minimal_subscriber
```

You should see the publisher logging messages, and the subscriber logging that it "heard" those messages.

#### `ros2 node`: Inspecting Nodes

The `ros2 node` command helps you inspect the active nodes in your ROS 2 graph.

-   `ros2 node list`: Lists all currently running nodes.
    ```bash
    ros2 node list
    # Expected output (after running publisher and subscriber):
    # /minimal_publisher
    # /minimal_subscriber
    ```

#### `ros2 topic`: Inspecting Topics

The `ros2 topic` command is used for interacting with and inspecting topics.

-   `ros2 topic list`: Lists all currently active topics.
    ```bash
    ros2 topic list
    # Expected output:
    # /parameter_events
    # /rosout
    # /topic  # This is our custom topic
    ```
-   `ros2 topic echo <topic_name>`: Displays messages being published on a specific topic.
    ```bash
    ros2 topic echo /topic
    # Expected output will be a stream of String messages from our publisher.
    ```
-   `ros2 topic info <topic_name>`: Provides information about a topic, including its type and the number of publishers/subscribers.
    ```bash
    ros2 topic info /topic
    # Expected output:
    # Type: std_msgs/msg/String
    # Publishers: 1
    # Subscribers: 1
    ```
### 3.7 Launch Files: Orchestrating Multiple Nodes

In real-world ROS 2 applications, you'll often have many nodes that need to start simultaneously and with specific configurations. Manually running each node in separate terminals can be cumbersome. This is where **launch files** come in.

A launch file is a convenient way to define and start multiple ROS 2 nodes, set their parameters, and even execute shell commands, all from a single command. ROS 2 primarily uses Python-based launch files.

Here's a simple Python launch file to start both our publisher and subscriber nodes:

```python
# src/examples/ch3_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_app', # The name of your ROS 2 package
            executable='minimal_publisher', # The executable name for the publisher node
            name='my_publisher_node', # Optional: give the node a custom name
            output='screen' # Display node output to the screen
        ),
        Node(
            package='my_robot_app', # The name of your ROS 2 package
            executable='minimal_subscriber', # The executable name for the subscriber node
            name='my_subscriber_node', # Optional: give the node a custom name
            output='screen' # Display node output to the screen
        )
    ])
```

**Explanation:**
-   `from launch import LaunchDescription`: Imports the main class to create a launch description.
-   `from launch_ros.actions import Node`: Imports the `Node` action, which represents a ROS 2 node to be launched.
-   `generate_launch_description()`: This function must be present in a Python launch file and returns a `LaunchDescription` object.
-   `LaunchDescription([...])`: Contains a list of actions to perform. Here, we're launching two `Node` actions.
-   `Node(...)`:
    -   `package`: The name of the ROS 2 package where the node is located.
    -   `executable`: The name of the Python script (without `.py` extension) that runs the node, as defined in `setup.py`.
    -   `name`: An optional, custom name for this specific instance of the node in the ROS graph.
    -   `output='screen'`: Directs the node's log output to the terminal where the launch file is run.

To run this launch file, you would typically use:

```bash
ros2 launch my_robot_app ch3_launch.py
```

*(Note: The exact executable names for `package` and `executable` in `ch3_launch.py` depend on how `setup.py` of `my_robot_app` is configured. Assuming `minimal_publisher` and `minimal_subscriber` are specified there.)*
### 3.8 Summary and Key Takeaways
### 3.9 Exercises
