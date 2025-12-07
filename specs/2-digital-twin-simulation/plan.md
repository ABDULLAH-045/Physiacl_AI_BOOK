# Plan for Module 2: The Digital Twin (Gazebo & Unity)

## 1. Scope and Dependencies

### In Scope:
*   Importing URDF models into Gazebo for basic physics simulation.
*   Adding and configuring virtual sensors (LiDAR, depth cameras, IMUs) in Gazebo.
*   Controlling simulated robots and receiving sensor data via ROS 2 bridge in Gazebo.
*   Configuring physics properties (gravity, friction, inertia) within Gazebo.
*   Setting up Unity environments for communication with ROS 2 via Unity Robotics Hub.
*   Building a complete digital twin pipeline integrating URDF, Gazebo, and Unity with ROS 2.
*   Creating high-fidelity, interactive environments in Unity for complex human-robot interaction scenarios.

### Out of Scope:
*   Advanced physics simulation features beyond basic kinematics and dynamics (e.g., fluid dynamics, soft body physics, deformable objects) unless explicitly required by user stories.
*   Development of new Gazebo plugins or Unity Robotics Hub features (focus on utilizing existing ones).
*   Real-world robot integration (this module focuses solely on simulation).
*   Detailed modeling of all possible sensor noise and inaccuracies beyond what standard plugins offer.
*   Creation of complex AI for robot autonomy (focus on enabling control and perception for testing AI).

### External Dependencies:
*   **ROS 2:** Core robotic operating system for inter-component communication.
*   **Gazebo:** Primary physics simulation environment.
*   **Unity:** High-fidelity 3D environment for visualization and interactive scenarios.
*   **Unity Robotics Hub:** Unity package for ROS 2 integration.
*   **URDF (Unified Robot Description Format):** Standard for robot modeling.
*   **RViz2:** ROS 2 visualization tool.
*   **Appropriate Gazebo sensor plugins:** (e.g., `gazebo_ros_pkgs` for LiDAR, depth camera, IMU).

## 2. Key Decisions and Rationale

1.  **Dual Simulation Environments (Gazebo & Unity):**
    *   **Decision:** Utilize both Gazebo and Unity for simulation, leveraging their respective strengths.
    *   **Rationale:** Gazebo excels in physics-accurate, real-time simulation, suitable for basic robot control, sensor modeling, and large-scale multi-robot scenarios. Unity offers superior visual fidelity, advanced rendering, and robust tools for creating interactive environments, making it ideal for human-robot interaction (HRI), complex scene design, and visually rich simulations.
    *   **Trade-offs:** This introduces complexity in managing two separate simulation environments and ensuring seamless data exchange between them via ROS 2. It requires familiarity with both platforms and their respective ROS 2 integration methods (e.g., `ros_gz_bridge` for Gazebo, Unity Robotics Hub for Unity).

2.  **ROS 2 as the Central Communication Backbone:**
    *   **Decision:** ROS 2 will serve as the sole communication middleware between all components (robot control, sensor data, simulation environments).
    *   **Rationale:** ROS 2 provides a standardized, asynchronous, and distributed communication framework. It allows for modular development, reusability of components, and a clear interface between the robot's "brain" and its "body" (simulated or real). Both Gazebo and Unity have mature ROS 2 integration packages.
    *   **Trade-offs:** Requires understanding of ROS 2 concepts (nodes, topics, services, actions) and careful design of message types and communication patterns to avoid bottlenecks and ensure real-time performance.

3.  **URDF as the Standard Robot Description Format:**
    *   **Decision:** All robot models will be defined using URDF.
    *   **Rationale:** URDF is the established standard for describing robot kinematics, dynamics, and visual properties in ROS-enabled ecosystems. It is well-supported by both Gazebo and Unity (via converters or direct import capabilities).
    *   **Trade-offs:** While URDF is powerful, creating complex robot models can be time-consuming. It may require additional tools for generating and visualizing URDFs.

4.  **Unity Robotics Hub for Unity-ROS 2 Integration:**
    *   **Decision:** Leverage the Unity Robotics Hub for connecting Unity scenes to ROS 2.
    *   **Rationale:** This official Unity package provides pre-built ROS 2 communication nodes, message serializers/deserializers, and tools to streamline the integration process, reducing development effort.
    *   **Trade-offs:** Dependencies on an external package, which may have its own release cycles and potential for breaking changes. Requires learning the specific API and conventions of the Unity Robotics Hub.

## 3. Interfaces and API Contracts

This section outlines the communication protocols and data exchange mechanisms between the various components of the digital twin simulation, primarily relying on ROS 2.

### 1. ROS 2 Topics (Publisher/Subscriber):
*   **Purpose:** Real-time, asynchronous data streams for sensor readings, joint states, command velocities, etc.
*   **Inputs:**
    *   Robot commands (e.g., `/cmd_vel` for differential drive, `/joint_command` for joint efforts/positions).
    *   Parameters for Gazebo (e.g., `/gazebo/set_model_state`).
    *   Unity HMI events (e.g., button presses, slider values).
*   **Outputs:**
    *   Sensor data (e.g., `/scan` for LiDAR, `/camera/depth/image_raw` for depth camera, `/imu/data` for IMU).
    *   Robot joint states (e.g., `/joint_states`).
    *   Gazebo model states (e.g., `/gazebo/model_states`).
    *   Unity visualization feedback (e.g., robot status, object positions).
*   **Versioning Strategy:** Adhere to ROS 2 message type evolution guidelines. Backward compatibility will be maintained where possible, and breaking changes will necessitate new message types or major version bumps.
*   **Idempotency:** N/A for most topic-based communication, as messages are often processed on arrival. Command topics should ideally be designed such that repeated identical messages don't cause unintended cumulative effects (e.g., setting a target velocity vs. adding a force increment).

### 2. ROS 2 Services (Request/Response):
*   **Purpose:** Synchronous, blocking calls for specific actions or queries (e.g., resetting simulation, spawning/despawning models, querying robot pose).
*   **Inputs:** Service requests (e.g., `gazebo_msgs/SpawnEntity`, `gazebo_msgs/SetModelState`).
*   **Outputs:** Service responses (e.g., success/failure status, queried data).
*   **Idempotency:** Services for setting state (e.g., `SetModelState`) should be idempotent. Services for triggering actions (e.g., `SpawnEntity`) may not be.

### 3. ROS 2 Actions (Goal/Feedback/Result):
*   **Purpose:** Long-running, cancellable operations with periodic feedback (e.g., robot navigation to a goal, complex manipulation tasks).
*   **Inputs:** Action goals (e.g., `nav2_msgs/NavigateToPose` for navigation).
*   **Outputs:** Action feedback (e.g., current pose during navigation), Action results (e.g., success/failure of navigation).
*   **Versioning Strategy:** Similar to topics, follow ROS 2 message type evolution guidelines.

### 4. Error Taxonomy & Handling:
*   **Gazebo/ROS 2 Bridge Errors:**
    *   Connection failures: Log and attempt reconnection.
    *   Invalid URDF: Simulation startup failure, provide clear error messages.
    *   Plugin errors: Log errors from Gazebo plugins.
    *   Communication timeouts: Implement retry mechanisms for services, mark topics as stale if no messages received.
*   **Unity Robotics Hub Errors:**
    *   ROS 2 node startup failures: Log and report to Unity console.
    *   Message parsing errors: Log and indicate corrupt data.
    *   Connection issues: Unity application should indicate disconnected status.
*   **General:** Use standard ROS 2 logging levels (DEBUG, INFO, WARN, ERROR, FATAL). Errors should be propagated appropriately to the user interface (e.g., RViz2, Unity scene overlay) or logging system.

## 4. Non-Functional Requirements (NFRs) and Budgets

This section details the quality attributes and constraints that the digital twin simulation system must satisfy.

### 1. Performance:
*   **Physics Simulation Rate (Gazebo):** Target 1x real-time factor (RTF) for typical humanoid robot simulations with basic sensors. For complex scenes or multiple robots, aim for at least 0.5x RTF.
*   **ROS 2 Latency:**
    *   **Control Commands:** p95 latency < 50 ms for joint commands to Gazebo/Unity.
    *   **Sensor Data:** p95 latency < 100 ms for LiDAR/depth camera data to RViz2/ROS 2 nodes.
*   **Throughput:** Gazebo and Unity ROS 2 bridges should handle typical sensor data rates (e.g., 10 Hz for LiDAR, 30 Hz for camera images) without dropping messages.
*   **Resource Caps:** Monitor CPU, GPU, and memory usage for both Gazebo and Unity processes. Provide recommendations for hardware specifications to achieve target performance.

### 2. Reliability:
*   **Simulation Stability:** Gazebo and Unity simulations should run for extended periods (e.g., 24 hours) without crashing or encountering significant physics anomalies (e.g., objects phasing through others, explosions).
*   **ROS 2 Connection Robustness:** The ROS 2 bridges for Gazebo and Unity should gracefully handle temporary disconnections and automatically attempt to reconnect without user intervention.
*   **Error Rate:** Target an error rate of < 0.1% for critical ROS 2 services (e.g., spawning models, resetting simulation).
*   **Degradation Strategy:** In case of performance degradation (e.g., low RTF), simulation fidelity might be reduced (e.g., lower sensor update rates, simpler physics models) to maintain stability.

### 3. Security:
*   **Access Control (ROS 2):** Utilize ROS 2 Security (SROS2) to implement authentication and authorization for critical topics and services, especially if simulations involve sensitive data or control.
*   **Data Handling:** Simulated sensor data should be treated as potentially sensitive, especially if it mimics real-world scenarios. Ensure data is not inadvertently exposed.
*   **Secrets Management:** Any API keys or credentials for external services (if introduced later) should be managed securely (e.g., environment variables, secret management systems).
*   **Auditing:** Log all significant simulation events and control commands for auditing and debugging purposes.

### 4. Cost:
*   **Hardware:** Recommend typical developer machine specifications (CPU, GPU, RAM) capable of running both Gazebo and Unity concurrently. Specify minimum requirements for basic usage.
*   **Software Licensing:** Acknowledge that Unity has licensing implications beyond personal use. Gazebo and ROS 2 are open source.
*   **Cloud Usage (if applicable later):** If cloud-based simulation is considered, provide estimates for compute, storage, and networking costs based on projected usage.

## 5. Data Management and Migration

This section outlines how data related to the digital twin simulation will be managed throughout its lifecycle, including configuration, schema evolution, and retention.

### 1. Source of Truth:
*   **Robot Models:** URDF files (and potentially SRDF for semantic descriptions) stored in a version-controlled repository (e.g., Git). These define the robot's kinematics, dynamics, visuals, and sensor attachments.
*   **Gazebo Worlds/Models:** `.world` files for environment descriptions and `.sdf` (Simulation Description Format) for individual Gazebo models, also managed in version control.
*   **Unity Scenes/Assets:** Unity project files and assets stored in a version-controlled repository (e.g., Git LFS for large assets).
*   **ROS 2 Configuration:** Launch files, parameter files (`.yaml`), and custom message/service/action definitions (`.msg`, `.srv`, `.action`) managed in version control within ROS 2 packages.
*   **Simulation Data (Logs/Recordings):** ROS 2 bags (`.db3` files for ROS 2 Foxy+), which record published topics, for post-analysis and debugging. These should be stored in a designated data directory.

### 2. Schema Evolution:
*   **URDF/SDF:** Changes to robot models or simulation environments should be versioned alongside the code. Backward compatibility should be considered, and significant changes might require updates to simulation launch files or controllers.
*   **ROS 2 Message Types:** Follow ROS 2 message type evolution guidelines. Avoid breaking changes if possible. If necessary, introduce new message types or update major versions of existing ones. Use aliases or deprecation warnings for smooth transitions.

### 3. Migration and Rollback:
*   **Configuration Files:** Leverage version control (Git) for all configuration files (URDF, SDF, launch files, Unity assets). Rollbacks can be achieved by reverting to previous commits.
*   **Simulation State:** Gazebo allows saving and loading world states, which can be used for checkpointing simulations or resuming from specific points.
*   **Unity Project:** Standard Unity project backup and version control practices should be followed.

### 4. Data Retention:
*   **ROS 2 Bags:** Retain simulation recordings (ROS 2 bags) for a defined period (e.g., 30 days, or indefinitely for critical test runs) for debugging, performance analysis, and regression testing. Implement automated cleanup policies for older data.
*   **Logs:** Retain system logs from Gazebo, Unity, and ROS 2 nodes for troubleshooting and auditing. Configure logging levels and rotation policies to manage storage.
*   **Derived Data:** Any generated data (e.g., analyzed sensor data, performance metrics) should have clear retention policies and storage locations.

## 6. Operational Readiness

This section addresses the considerations for ensuring the digital twin simulation system can be effectively operated, monitored, and maintained.

### 1. Observability:
*   **Logs:** All ROS 2 nodes (including Gazebo and Unity bridges), Gazebo processes, and Unity applications should output structured logs (e.g., JSON format) to a centralized logging system (e.g., ELK stack, Grafana Loki). Log levels (DEBUG, INFO, WARN, ERROR, FATAL) should be configurable.
*   **Metrics:** Key performance indicators (KPIs) should be collected and monitored.
    *   **Gazebo:** Real-time factor (RTF), simulation steps per second, physics update rate, model count, sensor update rates.
    *   **ROS 2:** Topic publication rates, subscription rates, message queue sizes, inter-node latency.
    *   **Unity:** Frame rate (FPS), script execution times, memory usage.
    *   **System:** CPU, GPU, RAM usage of host machine.
*   **Traces:** Distributed tracing (e.g., OpenTelemetry) can be implemented for complex multi-node interactions to track message flow and identify performance bottlenecks across ROS 2 nodes, Gazebo, and Unity.

### 2. Alerting:
*   **Thresholds:** Define alerts for critical conditions:
    *   RTF below acceptable thresholds (e.g., < 0.5 for extended periods).
    *   ROS 2 topic publication/subscription failures or significant latency spikes.
    *   Gazebo or Unity application crashes.
    *   High CPU/GPU/memory usage.
*   **On-Call Owners:** Clearly define who is responsible for responding to different types of alerts.

### 3. Runbooks:
*   Create clear, concise runbooks for common operational tasks and troubleshooting scenarios:
    *   **Starting/Stopping Simulation:** Step-by-step guide for launching Gazebo, Unity, and all necessary ROS 2 nodes.
    *   **Debugging Crashes:** Instructions for collecting logs, core dumps, and other diagnostic information.
    *   **Performance Troubleshooting:** Guide for identifying and resolving performance bottlenecks (e.g., high CPU usage, low RTF).
    *   **ROS 2 Network Issues:** Steps to diagnose and resolve communication problems.
    *   **URDF/SDF Validation:** Procedures for validating robot models and world files.

### 4. Deployment and Rollback strategies:
*   **Containerization:** Package Gazebo and ROS 2 components into Docker containers for consistent deployment environments and simplified dependency management. Unity applications can also be built into executables for deployment.
*   **Orchestration:** Use tools like Docker Compose or Kubernetes for managing and deploying multi-component simulations, especially for complex scenarios or cloud environments.
*   **Rollback:** Maintain version-controlled configurations (Git) for all deployment artifacts (Dockerfiles, orchestration manifests, launch files). In case of issues, a rollback to a previous working version should be achievable by deploying an older commit.

### 5. Feature Flags:
*   Implement feature flags for new or experimental simulation features (e.g., new sensor models, different physics engines, alternative control algorithms). This allows for A/B testing, gradual rollout, and quick toggling off of problematic features without redeploying the entire system.

## 7. Risk Analysis and Mitigation

This section identifies potential risks associated with the digital twin simulation module and outlines strategies to mitigate them.

### 1. Risk: Performance Degradation (Low RTF, High Latency)
*   **Description:** The simulation environments (Gazebo, Unity) or ROS 2 communication might not meet the required real-time factor or latency budgets, leading to unrealistic behavior or unresponsive control.
*   **Impact:** Unusable simulation for control development, inaccurate sensor data, difficulty in testing real-time algorithms.
*   **Mitigation:**
    *   **Optimize Models:** Simplify URDF/SDF models (reduce polygon count, joint complexity), optimize Unity assets.
    *   **Hardware Scaling:** Recommend specific CPU, GPU, and RAM configurations. Consider cloud-based high-performance computing (HPC) for demanding simulations.
    *   **Simulation Parameters:** Tune Gazebo physics engine parameters, reduce sensor update rates if necessary.
    *   **ROS 2 Optimization:** Use efficient message types, optimize network configuration, consider dedicated network interfaces.
    *   **Profiling:** Regularly profile simulation performance to identify bottlenecks (e.g., using `perf`, `gprof`, Unity Profiler, Gazebo's built-in profiler).

### 2. Risk: Integration Complexity between Gazebo, Unity, and ROS 2
*   **Description:** Challenges in setting up and maintaining robust communication and synchronization between two distinct simulation environments and the ROS 2 middleware.
*   **Impact:** Increased development time, brittle system, difficult debugging, limited functionality.
*   **Mitigation:**
    *   **Standardized Interfaces:** Strictly adhere to ROS 2 message types, services, and actions.
    *   **Modular Design:** Keep Gazebo, Unity, and ROS 2 components loosely coupled.
    *   **Automated Setup:** Develop scripts (e.g., Docker Compose) for automated deployment and configuration of the entire simulation stack.
    *   **Thorough Testing:** Implement integration tests to verify data flow and synchronization.

### 3. Risk: Inaccurate Physics or Sensor Modeling
*   **Description:** The simulated physics or sensor data might not accurately reflect real-world conditions, leading to control algorithms or perception systems that perform poorly when deployed on real robots.
*   **Impact:** Negative transfer learning from simulation to real world, wasted development effort, potential safety risks.
*   **Mitigation:**
    *   **Parameter Tuning:** Carefully calibrate Gazebo physics parameters (friction, damping, restitution) against real-world data.
    *   **Sensor Model Validation:** Compare simulated sensor outputs with real sensor data in controlled environments.
    *   **Domain Randomization:** Introduce variability in simulation parameters (e.g., textures, lighting, object properties) to make models more robust to real-world variations.
    *   **Continuous Feedback Loop:** Establish a process for regularly comparing simulation results with real-world robot behavior and updating models accordingly.

### 4. Risk: Unity Licensing and Cost Implications
*   **Description:** Reliance on Unity, which is commercial software, might introduce licensing costs or restrictions for certain use cases (e.g., commercial products, large teams).
*   **Impact:** Unexpected project costs, legal issues, inability to scale.
*   **Mitigation:**
    *   **Early License Assessment:** Clearly define the project's long-term commercial goals and assess Unity's licensing terms.
    *   **Alternative Consideration:** If licensing becomes a major constraint, investigate open-source alternatives for high-fidelity visualization (e.g., Godot, Unreal Engine with custom ROS integration) for future phases, although this would impact the current plan.
    *   **Budget Allocation:** Ensure that potential Unity licensing costs are factored into the project budget.

## 8. Evaluation and Validation

This section defines the criteria for determining the successful completion of the module and the methods for verifying its functionality and performance.

### 1. Definition of Done (DoD):
*   All user stories (Simulating Humanoid in Gazebo, Adding Virtual Sensors, Creating Interactive Scene in Unity) from the `spec.md` are implemented and meet their respective acceptance criteria.
*   All functional requirements (FR-001 to FR-006) are met.
*   Key architectural decisions are documented (e.g., in this plan and potentially dedicated ADRs).
*   Integration tests pass for communication between Gazebo, Unity, and ROS 2.
*   Performance metrics (RTF, latency) are within acceptable NFR ranges during testing.
*   Basic operational readiness (logging, monitoring) is established.
*   Code is reviewed, documented, and adheres to project coding standards.

### 2. Tests:
*   **Unit Tests:** For individual ROS 2 nodes, custom Unity scripts, or helper libraries, ensuring correctness of algorithms and logic.
*   **Integration Tests:**
    *   **Gazebo-ROS 2 Bridge:** Verify correct publication/subscription of joint commands, sensor data, and service calls.
    *   **Unity-ROS 2 Bridge:** Verify correct communication via Unity Robotics Hub for robot control and scene interactions.
    *   **End-to-End Simulation Tests:** Validate complete user scenarios, e.g., "robot navigates to a target in Unity scene based on ROS 2 command."
*   **Performance Tests:** Measure RTF, latency, and resource utilization under various simulation loads.
*   **Regression Tests:** Ensure that new changes do not break existing functionality or introduce performance regressions.

### 3. Output Validation:
*   **URDF/SDF Validation:** Use tools like `check_urdf` or Gazebo's model validation to ensure robot and world descriptions are syntactically correct and physically sound.
*   **Sensor Data Validation:** Compare simulated sensor output (e.g., LiDAR scans, depth images) against ground truth or expected values within the simulation. Visualize sensor data in RViz2 to visually confirm correctness.
*   **Robot Behavior Validation:** Observe robot movement and interactions in both Gazebo and Unity to ensure they conform to expected physics and control commands.
*   **ROS 2 Message Validation:** Monitor ROS 2 topics using `ros2 topic echo` and `ros2 interface show` to confirm message types and content are as expected.
*   **User Story Acceptance Scenarios:** Directly translate the acceptance scenarios from the `spec.md` into test cases, providing clear steps and expected outcomes for manual or automated verification.
    *   *SC-001:* Load URDF into Gazebo, robot stands without falling.
    *   *SC-002:* Simulate LiDAR, visualize in RViz2.
    *   *SC-003:* Complete final mini-project ("Simulated Warehouse Worker").
    *   *SC-004:* Unity robot navigates and interacts with objects via ROS 2.

## 9. Architectural Decision Record (ADR)

Any architecturally significant decisions made during the implementation of this module that deviate from, or expand upon, the decisions outlined in this plan should be documented in a separate Architectural Decision Record (ADR). This ensures a clear record of the decision, its context, alternatives considered, and the rationale behind the chosen approach.
