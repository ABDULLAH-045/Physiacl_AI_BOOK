# Tasks for Module 2: The Digital Twin (Gazebo & Unity)

This document outlines the detailed tasks required to implement Module 2, covering the integration of Gazebo, Unity, and ROS 2 for digital twin simulation. Tasks are grouped by user story and functional requirement for clarity and traceability.

---

## User Story 1: Simulating a Humanoid in Gazebo (P1)
*As a student, I want to import my humanoid robot's URDF into Gazebo and simulate its basic physics, so that I can test its stability and movement in a virtual environment.*


### FR-001: The student MUST be able to import a URDF model into Gazebo and simulate its physics.

1.  **Task: Set up a ROS 2 package for robot description.**
    *   Create a new ROS 2 `ament_python` or `ament_cmake` package (e.g., `humanoid_description`).
    *   Add necessary dependencies (e.g., `urdf_parser_py`, `xacro`).
    *   **Acceptance Test**: Package builds successfully with `colcon build`.

2.  **Task: Create a basic humanoid URDF model.**
    *   Design a simple URDF file (or Xacro macro) for a humanoid robot with a few links and joints (e.g., torso, head, two arms, two legs).
    *   Define basic visual and collision geometries.
    *   Define inertial properties for each link.
    *   **Acceptance Test**: URDF can be successfully parsed and viewed using `check_urdf` and `urdf_to_graphiz`.

3.  **Task: Integrate URDF with Gazebo.**
    *   Create a Gazebo launch file that loads the humanoid URDF model into an empty Gazebo world.
    *   Ensure the URDF includes `gazebo_ros2_control` tags for future control.
    *   **Acceptance Test**: Launch file executes, and the humanoid model appears in Gazebo without errors (e.g., no "missing physics plugin" warnings).
    *   **Acceptance Test**: (SC-001) The robot model stands without immediately falling over due to incorrect physics properties.

4.  **Task: Implement basic physics control via ROS 2-Gazebo bridge.**
    *   Use `ros_gz_bridge` or similar to enable communication between Gazebo and ROS 2.
    *   Develop a simple ROS 2 node (e.g., in Python or C++) to publish joint torque commands to the simulated robot's joints.
    *   **Acceptance Test**: (Acceptance Scenario 2) Applying a torque to a joint using the ROS 2 node results in observable movement of the corresponding link in Gazebo.

---

## User Story 2: Adding Virtual Sensors to the Robot (P2)
*As a student, I want to add and configure virtual sensors like LiDAR and depth cameras to my robot in Gazebo, so that I can test perception algorithms.*

### FR-002: The system MUST provide plugins for simulating common robot sensors, including LiDAR, depth cameras, and IMUs.

5.  **Task: Add a simulated LiDAR sensor to the URDF.**
    *   Modify the humanoid URDF to include a `gazebo` sensor tag for a LiDAR (e.g., using `libgazebo_ros_ray_sensor.so`).
    *   Configure its properties (e.g., min/max angle, range, resolution, update rate).
    *   **Acceptance Test**: Robot loads in Gazebo, and a `/scan` topic is published.

6.  **Task: Visualize LiDAR data in RViz2.**
    *   Launch RViz2 and configure a `LaserScan` display to subscribe to the `/scan` topic.
    *   Place objects in the Gazebo world.
    *   **Acceptance Test**: (SC-002, Acceptance Scenario 1) LiDAR data is visualized correctly in RViz2, showing objects in the Gazebo world.

7.  **Task: Add a simulated Depth Camera sensor to the URDF.**
    *   Modify the humanoid URDF to include a `gazebo` sensor tag for a depth camera (e.g., using `libgazebo_ros_depth_camera.so`).
    *   Configure its properties (e.g., horizontal/vertical FOV, resolution, update rate).
    *   **Acceptance Test**: Robot loads in Gazebo, and `image_raw`, `depth/image_raw`, and `points` topics are published.

8.  **Task: Visualize Depth Camera data in RViz2.**
     *   Launch RViz2 and configure `Image` and `PointCloud2` displays to subscribe to the camera topics.
    *   **Acceptance Test**: (Acceptance Scenario 2) Depth camera images and point clouds are visualized correctly in RViz2.

9.  **Task: Add a simulated IMU sensor to the URDF.**
    *   Modify the humanoid URDF to include a `gazebo` sensor tag for an IMU (e.g., using `libgazebo_ros_imu_sensor.so`).
    *   Configure its properties (e.g., update rate, noise parameters).
    *   **Acceptance Test**: Robot loads in Gazebo, and an `/imu/data` topic is published.

---

## User Story 3: Creating an Interactive Scene in Unity (P3)
*As a student, I want to build a high-fidelity, interactive environment in Unity and connect it to my ROS 2 system, so that I can create complex human-robot interaction scenarios.*

### FR-005: The student MUST be able to set up a Unity environment that communicates with ROS 2 via the Unity Robotics Hub.

10. **Task: Set up a new Unity project for robotics simulation.**
    *   Create a new 3D Unity project.
    *   Import the Unity Robotics Hub package from the Unity Asset Store or GitHub.
    *   **Acceptance Test**: Unity project opens, and Robotics Hub tools are available in the editor.

11. **Task: Configure Unity to communicate with ROS 2.**
    *   Set up ROS 2 communication in Unity using the ROS-TCP-Connector.
    *   Create a simple Unity script to publish and subscribe to a test ROS 2 topic (e.g., `std_msgs/String`).
    *   **Acceptance Test**: Unity application can send and receive messages from a basic ROS 2 listener/publisher node.

12. **Task: Import the humanoid robot model into Unity.**
    *   Use the Unity Robotics Hub URDF Importer to import the previously created humanoid URDF model into the Unity scene.
    *   **Acceptance Test**: Humanoid model appears in Unity with correct joint hierarchy and visuals.

13. **Task: Implement basic robot control in Unity via ROS 2.**
    *   Create Unity scripts to subscribe to ROS 2 command topics (e.g., `/cmd_vel` or joint command topics).
    *   Translate ROS 2 commands into Unity physics/joint movements.
    *   **Acceptance Test**: (Acceptance Scenario 1) Controlling the robot via a ROS 2 topic results in observable movement in the Unity scene.

14. **Task: Implement sensor data publishing from Unity (optional/advanced).**
    *   If Unity is to simulate its own sensors (e.g., higher fidelity cameras), create Unity scripts to publish `sensor_msgs` to ROS 2.
    *   **Acceptance Test**: Unity publishes simulated sensor data that can be visualized in RViz2 or processed by other ROS 2 nodes.

---

## Overall Integration & Pipeline (FR-006)
*The student MUST be able to build a complete digital twin pipeline, integrating URDF, Gazebo, and Unity with ROS 2.*

15. **Task: Develop a comprehensive launch system for the digital twin pipeline.**
    *   Create ROS 2 launch files (`.launch.py` or `.launch.xml`) to orchestrate the startup of:
        *   Gazebo with the humanoid robot and sensors.
        *   `ros_gz_bridge` nodes.
        *   Any ROS 2 control nodes.
        *   (Optional) The Unity application instance.
    *   **Acceptance Test**: A single launch command successfully brings up the entire integrated Gazebo-ROS 2 simulation.

16. **Task: Document the complete digital twin setup process.**
    *   Update the `quickstart.md` in `specs/2-digital-twin-simulation/` (or create a new `README.md` if preferred) with step-by-step instructions for setting up and running the full digital twin.
    *   Include instructions for both Gazebo and Unity components.
    *   **Acceptance Test**: A new user can follow the documentation to replicate the full digital twin environment.

17. **Task: Implement the "Simulated Warehouse Worker" mini-project (SC-003, SC-004).**
    *   Create a simple warehouse-like environment in Unity (e.g., a few shelves, a box).
    *   Develop ROS 2 nodes for navigation goals.
    *   Implement basic interaction (e.g., robot "picks up" a virtual box when it reaches it).
    *   **Acceptance Test**: The robot navigates to a target and interacts with an object in the Unity scene based on ROS 2 commands.

---

## Testing & Validation Tasks

18. **Task: Develop unit tests for custom ROS 2 nodes and Unity scripts.**
    *   Write tests for any custom logic developed (e.g., control algorithms, sensor processing).
    *   **Acceptance Test**: All unit tests pass.

19. **Task: Implement integration tests for ROS 2 communication.**
    *   Test communication between ROS 2 nodes, Gazebo, and Unity.
    *   **Acceptance Test**: Integration tests verify correct message passing and command execution.

20. **Task: Perform performance benchmarking.**
    *   Measure RTF, latency, and resource usage for various simulation scenarios.
    *   Identify and address performance bottlenecks.
    *   **Acceptance Test**: Performance metrics meet NFRs (e.g., RTF > 0.5, latency < 100ms).

---

## Documentation & Maintenance Tasks

21. **Task: Review and refine `spec.md`, `plan.md`, `data-model.md`, and `research.md`.**
    *   Ensure all documents are consistent and up-to-date with the implemented features.
    *   **Acceptance Test**: Documents reflect the final implementation accurately.

22. **Task: Add module 2 content to Docusaurus.**
    *   Create `index.mdx` and `tutorial.mdx` under `physical-ai-humanoid-robotics-book/docs/2-digital-twin-simulation/`.
    *   Populate with introductory content and a walkthrough of the module.
    *   **Acceptance Test**: Docusaurus site builds and serves correctly, showing the new module in the sidebar.

23. **Task: Create a CHANGELOG entry for Module 2.**
    *   Document the completion of Module 2 in `CHANGELOG.md`.
    *   **Acceptance Test**: `CHANGELOG.md` is updated.