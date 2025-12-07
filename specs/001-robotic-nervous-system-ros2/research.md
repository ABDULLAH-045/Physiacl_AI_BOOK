
# Research Plan: Module 1
This document outlines the research required to resolve open questions for the development of Module 1.

## 1. ROS 2 QoS Settings for Real-time Performance

**Decision**: For most examples, the default QoS settings for publishers and subscriptions (`Reliable`, `Volatile`, `History Depth: 10`) are sufficient. For the final project connecting to an AI agent, a custom QoS profile will be used: `Best Effort` for high-frequency sensor data and `Reliable` for critical commands, both with a history depth of 1.

**Rationale**: This balances performance and reliability. `Best Effort` is suitable for high-frequency data where occasional packet loss is acceptable (e.g., sensor streams for visualization), while `Reliable` ensures that critical commands (e.g., "move arm") are not lost. A small history depth minimizes latency.

**Alternatives considered**:
-   Using `Reliable` for all topics: Simpler, but can introduce latency and performance issues with high-frequency data.
-   Using a custom C++ executor with real-time priority: More complex and beyond the scope of an introductory module.

## 2. Node Communication Strategies

**Decision**: The primary communication patterns will be topics for streaming data (sensor values, joint states) and services for request/response interactions (e.g., toggle LED). Actions will be introduced for long-running tasks like "move arm to position".

**Rationale**: This aligns with standard ROS 2 best practices and provides students with a clear understanding of when to use each communication type.

**Alternatives considered**:
-   Using only topics: Possible, but requires implementing custom request/reply logic on top of topics, which is less efficient and robust than using services.
-   Using services for all interactions: Not suitable for streaming data.

## 3. Hardware Configurations for Examples

**Decision**: All examples will be designed to run in a simulated environment that can be executed on a standard PC (Linux VM). A separate appendix will provide guidance for deploying to a physical Jetson Orin with a common sensor (e.g., Realsense D435) and a simple robotic arm (e.g., WidowX 250).

**Rationale**: This ensures the core content is accessible to all students without requiring specific hardware. The appendix provides a path for students who wish to apply their knowledge to a physical system.

**Alternatives considered**:
-   Requiring specific hardware: This would create a barrier to entry.
-   Providing multiple hardware configurations: This would add significant complexity to the module.
