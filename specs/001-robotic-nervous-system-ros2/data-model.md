# Data Model: Module 1

This document defines the key data entities for the ROS 2 module. As this module is focused on the robotics middleware itself, the "data model" refers to the core concepts of ROS 2 that students will be working with.

## 1. ROS 2 Node

-   **Description**: A process that performs computation. The fundamental executable unit in ROS 2.
-   **Attributes**:
    -   `name`: A unique identifier for the node within the ROS graph.
    -   `namespace`: An optional prefix to group related nodes.
-   **Relationships**:
    -   Can have multiple `Publishers`, `Subscribers`, `Service Servers`, `Service Clients`, `Action Servers`, and `Action Clients`.

## 2. Topic

-   **Description**: A named bus for messages. Nodes publish messages to topics, and other nodes subscribe to topics to receive those messages.
-   **Attributes**:
    -   `name`: The unique name of the topic (e.g., `/joint_states`).
    -   `message_type`: The data type of messages on the topic (e.g., `sensor_msgs/msg/JointState`).

## 3. Message

-   **Description**: The data structure for communication. Messages are defined in `.msg` files.
-   **Attributes**: A collection of typed fields (e.g., `string`, `float64[]`, `Header`).

## 4. Service

-   **Description**: A request/response communication pattern.
-   **Attributes**:
    -   `name`: The unique name of the service.
    -   `service_type`: The data type for the request and response, defined in a `.srv` file.

## 5. Action

-   **Description**: A communication pattern for long-running, preemptible tasks that provide feedback.
-   **Attributes**:
    -   `name`: The unique name of the action.
    -   `action_type`: The data type for the goal, result, and feedback, defined in an `.action` file.

## 6. URDF Model

-   **Description**: An XML file format for representing a robot model.
-   **Key Components**:
    -   `link`: Describes a rigid body part of the robot.
    -   `joint`: Describes the kinematics and dynamics of the connection between links.
    -   `visual`: Defines the visual representation of a link.
    -   `collision`: Defines the collision geometry of a link.
