# Chapter 2: ROS 2 Communication Mechanisms: Speaking the Robot's Language

## Purpose:
To detail the primary communication patterns in ROS 2 and their appropriate use cases.

## Learning Objectives:
- Differentiate between topics, services, and actions.
- Understand Quality of Service (QoS) settings and their impact on communication reliability and performance.
- Learn about parameters for dynamic node configuration.
- Grasp the role of messages and interfaces (`.msg`, `.srv`, `.action` files).

## Expected Outputs:
- Understanding of when to use topics, services, or actions.
- Ability to interpret basic ROS 2 CLI communication commands.
- Simple code snippets (Python preferred for initial clarity) demonstrating message publication/subscription, service calls.

## Content Outline:
### 2.1 Recap of Core Communication Concepts

In Chapter 1, we were introduced to the fundamental communication mechanisms in ROS 2: Topics, Services, and Actions. While they all facilitate inter-node communication, they are designed for distinct interaction patterns and use cases. Choosing the right communication mechanism is crucial for efficient and robust ROS 2 applications.

| Communication Mechanism | Interaction Pattern               | Use Case Examples                                  | Key Characteristics                                     |
| :---------------------- | :-------------------------------- | :------------------------------------------------- | :------------------------------------------------------ |
| **Topics**              | Publish/Subscribe (Asynchronous)  | Sensor data streams (e.g., camera, lidar), continuous commands (e.g., motor velocities), logging | Anonymous, one-to-many, fire-and-forget, continuous data flow |
| **Services**            | Request/Reply (Synchronous)       | One-shot commands (e.g., "open gripper"), parameter retrieval, configuration changes, specific calculations | Direct client-server, blocking, immediate result, single request/response |
| **Actions**             | Goal/Feedback/Result (Asynchronous)| Navigation to a goal, manipulating an arm, complex sensing sequences | Long-running tasks, periodic feedback, cancellable, stateful |

We will now dive deeper into each of these communication primitives, exploring their mechanics, associated message types, and how to interact with them using ROS 2 command-line tools.
### 2.2 Deep Dive into Topics: Anonymous Data Streams
#### 2.2.1 Publishers and Subscribers
#### 2.2.2 Message Types

Data exchanged over topics (and used in services and actions) is structured using **message types**. These are defined in `.msg` files, which specify the name and type of each field the message contains. ROS 2 provides a wide array of standard message types (e.g., `std_msgs`, `geometry_msgs`, `sensor_msgs`) for common data, but you can also define custom message types.

**Example `std_msgs/msg/String.msg`:**
```
string data
```

**Example `geometry_msgs/msg/Point.msg`:**
```
float64 x
float64 y
float64 z
```

Understanding message types is crucial because they form the contract for data exchange. Both the publisher and subscriber must agree on the message type being used.
#### 2.2.3 Basic CLI Tools for Topics (`ros2 topic`)
### 2.3 Services: Synchronous Request/Reply
#### 2.3.1 Service Servers and Clients
#### 2.3.2 Service Definition Files (`.srv`)

Services are defined using `.srv` files, which are similar to `.msg` files but contain two parts: a request and a response, separated by a `---` line. The request defines the input parameters for the service call, and the response defines the output parameters.

**Example `example_interfaces/srv/AddTwoInts.srv`:**
```
int64 a
int64 b
---
int64 sum
```

When a client calls this service with two integers (`a` and `b`), the server performs the addition and returns the `sum`. This explicit contract ensures that both client and server understand the expected data for interaction.
#### 2.3.3 Basic CLI Tools for Services (`ros2 service`)
### 2.4 Actions: Long-Running Goals with Feedback
#### 2.4.1 Action Servers and Clients
#### 2.4.2 Action Definition Files (`.action`)

Actions are the most complex communication primitive and are defined using `.action` files. An `.action` file is composed of three parts, separated by `---` lines: the Goal, the Result, and the Feedback.

**Example `example_interfaces/action/Fibonacci.action`:**
```
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] sequence
```

-   The **Goal** defines the input to the action server (e.g., `order` for Fibonacci sequence).
-   The **Result** defines the final output returned when the action completes (e.g., the `sequence`).
-   The **Feedback** defines the type of messages sent periodically by the action server to the client to indicate progress (e.g., the `sequence` being built).

This rich definition allows for robust and interactive control of long-running tasks.
### 2.5 Parameters: Dynamic Configuration

Parameters in ROS 2 allow nodes to expose configurable values that can be changed at runtime without recompiling or restarting the node. This provides immense flexibility for tuning robot behavior, adapting to different environments, or adjusting sensor thresholds.

#### 2.5.1 Setting and Getting Parameters

-   **Declaration:** Nodes declare their parameters with default values and types (e.g., integer, float, string, boolean).
-   **Getting Parameters:** Nodes can retrieve the current value of their declared parameters.
-   **Setting Parameters:** Parameters can be changed by other nodes or by the user via command-line tools.

#### 2.5.2 Basic CLI Tools for Parameters (`ros2 param`)

The `ros2 param` command-line tool is essential for inspecting and modifying parameters:

-   `ros2 param list`: Lists all parameters currently exposed by active nodes.
-   `ros2 param get <node_name> <parameter_name>`: Retrieves the current value of a specific parameter from a node.
-   `ros2 param set <node_name> <parameter_name> <value>`: Sets the value of a specific parameter on a node.
-   `ros2 param dump <node_name>`: Dumps all parameters of a node to a YAML file, useful for saving configurations.
### 2.6 Quality of Service (QoS): Controlling Communication Behavior

ROS 2's underlying DDS layer provides powerful mechanisms to fine-tune how messages are exchanged between nodes. These mechanisms are exposed as **Quality of Service (QoS)** policies, allowing developers to control aspects like reliability, message history, and data persistence. Choosing the right QoS profile is crucial for ensuring that your ROS 2 application behaves as expected, especially in real-time or resource-constrained environments.

#### 2.6.1 Key QoS Policies

-   **Reliability:** This policy determines whether message delivery is guaranteed.
    -   **Best Effort:** Messages are sent as quickly as possible, but there's no guarantee of delivery. If the network is congested or a subscriber is not ready, messages might be lost. This is suitable for high-frequency, non-critical data (e.g., sensor readings where missing a few updates is acceptable).
    -   **Reliable:** Messages are guaranteed to be delivered. The DDS layer will retransmit messages if necessary to ensure they reach all subscribers. This is critical for important data (e.g., command signals, configuration updates) where loss is unacceptable.

-   **Durability:** This policy specifies whether messages persist for new subscribers.
    -   **Volatile:** Messages are only available to subscribers that are active when the message is published. New subscribers will not receive past messages.
    -   **Transient Local:** The publisher will keep a history of messages and resend them to new subscribers. This is useful for providing context to nodes that join later.

-   **History:** This policy defines how many messages the publisher (and often the subscriber) will store.
    -   **Keep Last:** Only the most recent 'N' messages are kept.
    -   **Keep All:** All messages up to a certain resource limit are kept.

-   **Liveliness:** This policy helps nodes detect whether their communication partners are still active.
    -   **Automatic:** The DDS implementation automatically asserts liveliness.
    -   **Manual By Topic:** Liveliness must be manually asserted by the application.

#### 2.6.2 Impact on Communication

The choice of QoS policies has a significant impact on:

-   **Performance:** Reliable communication typically has higher overhead due to retransmissions and acknowledgements. Best Effort is faster but less guaranteed.
-   **Resource Usage:** Storing message history (Transient Local durability, Keep All history) consumes memory resources.
-   **System Robustness:** Correct QoS settings can make your system more resilient to network issues or node restarts.

By understanding and appropriately configuring QoS policies, you can optimize your ROS 2 applications for factors like latency, throughput, and data integrity.
### 2.7 Summary and Key Takeaways
### 2.8 Exercises
