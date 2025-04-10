# ROS 1 to ROS 2 Migration: Challenges and Solutions for Butler Swarm Simulation

## Introduction

This document details the significant challenges encountered and solutions implemented during the migration of the Butler Swarm simulation project from a ROS 1 legacy structure to ROS 2 Jazzy, targeting the Gazebo Harmonic simulation environment. The migration aimed to leverage ROS 2's advancements in performance, architecture, and tooling while maintaining the core swarm robotics functionalities. The process involved substantial refactoring of Python code, launch configurations, build systems, and simulation model definitions, often requiring iterative debugging across the ROS 2 and Gazebo ecosystems.

## Key Migration Challenges (Recent First)

### 1. Gazebo Harmonic Sensor Simulation in Virtualized Environments (UTM on Apple Silicon)

A major roadblock emerged when attempting to simulate sensor functionality, specifically the Lidar, within the chosen development environment (Ubuntu 24.04 VM managed by UTM on Apple Silicon macOS).

*   **Initial Goal:** Utilize the `gpu_lidar` sensor type provided by Gazebo Sim for efficient, hardware-accelerated laser scan generation, crucial for multi-agent simulation performance.
*   **Problem Encountered:** Persistent failures during simulation launch, including:
    *   `OGRE EXCEPTION(3:RenderingAPIException): OpenGL 3.3 is not supported.` errors from the Gazebo server (`gzserver`).
    *   `OGRE EXCEPTION(3:RenderingAPIException): Vertex Program Ogre/... failed to compile.` errors from the Gazebo client (`gzclient`).
    *   RViz rendering conflicts (`Window with name 'OgreWindow(0)' already exists`).
    *   Absence of scan data on expected Gazebo and ROS topics, despite the sensor entity appearing in the Gazebo simulation tree.
*   **Investigation & Debugging Steps:**
    *   **VM Graphics Settings:** Confirmed UTM Hardware Acceleration was enabled and experimented with `virtio-gpu-gl-device` and `virtio-gpu-gl-pci` display types.
    *   **Guest OS Drivers:** Updated Mesa graphics drivers within the Ubuntu VM using `sudo apt update && sudo apt upgrade`. Checked the reported OpenGL version via `glxinfo | grep "OpenGL version"`, confirming it remained below the required 3.3 threshold.
    *   **External Research:** Consulted UTM documentation and GitHub issues (e.g., [#4285](https://github.com/utmapp/UTM/issues/4285)), confirming a known limitation where the ANGLE translation layer used by UTM on macOS restricts the guest VM's OpenGL version (typically to 2.1 or ES 3.0), irrespective of host capabilities.
    *   **Render Engine Switching:** Attempted forcing Gazebo to use the older OGRE 1 engine via the `GZ_SIM_RENDER_ENGINE=ogre` environment variable and the `--render-engine ogre` launch argument. While this bypassed the OpenGL 3.3 requirement, it introduced RViz conflicts and different shader compilation errors, indicating persistent graphics incompatibility.
    *   **Sensor Type Switching:** Attempted switching from `gpu_lidar` to the CPU-based `ray` sensor type in the URDF. Encountered Gazebo warnings (`Sensor type LIDAR not supported yet. Try using a GPU LIDAR instead.`) when combined with OGRE 1, suggesting an incompatibility between the `ray` sensor's `<lidar>` configuration block and the OGRE 1 engine in Gazebo Harmonic.
    *   **SDF Tag Validation:** Identified and removed an invalid `<gz_frame_id>` tag used within the `<sensor>` definition based on Gazebo warnings. Confirmed sensor frame is typically inherited from the parent link.
*   **Conclusion & Solution:** The underlying limitation of OpenGL version support within the UTM VM on Apple Silicon prevents the successful operation of the `gpu_lidar` sensor. Attempts to use alternative configurations (`ray` sensor, OGRE 1) also failed due to specific incompatibilities. To proceed with other aspects of the simulation, the Lidar sensor functionality was temporarily disabled by commenting out the sensor definition in the URDF and the corresponding bridging rule in the `ros_gz_bridge` configuration. Functional lidar simulation would require a different virtualization solution with better GPU passthrough or running natively on a Linux system.
*   **Academic Relevance:** This highlights the critical impact of the underlying execution environment (virtualization, host OS, hardware) on simulation fidelity and feature support. It underscores the importance of understanding graphics pipelines (OpenGL versions, rendering engines like OGRE, translation layers like ANGLE) and driver compatibility when working with GPU-accelerated simulation components. Debugging involved interpreting specific error messages, consulting external documentation and community issues, and systematically isolating variables (render engine, sensor type).

### 2. Gazebo Topic Namespacing & `ros_gz_bridge` Configuration

Ensuring correct data flow between ROS 2 nodes and Gazebo plugins proved challenging due to topic naming conventions and bridge behavior.

*   **Problem Encountered:** Sensors (Odometry, Lidar, Joint States) appeared to load in Gazebo, and corresponding ROS topics were created by the bridge, but no data was received on the ROS side (`ros2 topic echo` showed nothing). Conversely, `cmd_vel` messages sent from ROS were initially not actuating the robot in Gazebo.
*   **Investigation & Debugging Steps:**
    *   **Topic Listing:** Compared the output of `ros2 topic list` (ROS view) and `gz topic -l` (Gazebo view) to identify available topics and their exact names.
    *   **Discrepancy Identification:** Found mismatches between the Gazebo topic names configured in the bridge (`gz_topic_name`) and the topics actually being published by the Gazebo plugins (verified using `gz topic -l` and `gz topic -e`). For example, the DiffDrive plugin published odometry on `/model/{namespace}/odometry`, while the bridge initially expected `/{namespace}/odometry`. The Lidar plugin published on `/scan` or `/{namespace}/scan` depending on configuration, while the bridge expected `/model/{namespace}/scan`.
    *   **Plugin Defaults:** Researched or deduced the default topic naming schemes for Gazebo plugins (e.g., DiffDrive often uses `/model/{model_name}/...`). Found that omitting the `<topic>` tag in the plugin configuration causes it to use these defaults.
    *   **Bridge Configuration (`gz_bridge_template.yaml`):** Iteratively modified the `gz_topic_name` values in the bridge template YAML file to precisely match the Gazebo topics identified via `gz topic -l`.
    *   **ROS Topic Naming:** Ensured all `ros_topic_name` entries in the bridge configuration used absolute paths (starting with `/`) to prevent double-namespacing issues caused by the bridge node itself running within the agent's namespace (e.g., preventing `/agent_0/agent_0/scan`).
    *   **Gazebo Topic Echoing:** Used `gz topic -e -t <gazebo_topic_name>` to confirm whether plugins were actually publishing data within Gazebo, helping to distinguish between a publishing failure and a bridging failure.
*   **Conclusion & Solution:** Consistent and accurate topic mapping is paramount for the `ros_gz_bridge`. The solution involved careful inspection of actual Gazebo topics using command-line tools and precisely aligning the `gz_topic_name` in the bridge configuration. Furthermore, ensuring absolute paths for `ros_topic_name` prevented unintended namespacing issues. The bridge template (`gz_bridge_template.yaml`) was used for dynamic configuration generation in the launch file, rendering the static `gz_bridge_config.yaml` unnecessary.
*   **Academic Relevance:** Demonstrates debugging techniques for distributed middleware systems (ROS-Gazebo interaction). Emphasizes the need to verify interfaces (topics) at each boundary. Highlights potential pitfalls in namespacing and the importance of understanding configuration file semantics for bridging tools.

### 3. XACRO Argument Processing and Namespacing in URDF

Incorrectly applying namespaces to links, joints, and sensors defined via XACRO macros was a significant source of errors, preventing parts of the robot model from loading correctly.

*   **Problem Encountered:** Components like the `lidar_link` were missing from the Gazebo entity tree. Sensor plugins failed to load correctly. Gazebo topics were missing expected namespaces (e.g., `/odom` instead of `/agent_0/odom`).
*   **Investigation & Debugging Steps:**
    *   **Launch File:** Confirmed the `xacro` command was invoked with the `namespace:=agent_X` argument.
    *   **Top-Level URDF (`butler_agent.urdf.xacro`):** Discovered it initially defined `<xacro:property name="namespace" value="" />`. This local property definition shadowed the `namespace` argument passed via the command line.
    *   **Included Macros (`*.urdf.xacro`):** Found that included files defining links/joints/sensors (e.g., `base`, `wheels`, `lidar`, `gazebo`) used the `${namespace}` syntax. This syntax accesses XACRO *properties*, which evaluated to the empty string due to the shadowing property in the top-level file.
    *   **XML Syntax:** Identified incorrect comment syntax (`#`) within XACRO files, causing XML parsing errors.
*   **Conclusion & Solution:** The distinction between XACRO arguments (`<xacro:arg>`) and properties (`<xacro:property>`) was critical. Arguments are passed via the command line or `include` tags, while properties are locally defined variables. To correctly pass the command-line namespace:
    1.  Replaced `<xacro:property name="namespace"...>` with `<xacro:arg name="namespace" default=""/>` in `butler_agent.urdf.xacro`.
    2.  Updated all macro calls in `butler_agent.urdf.xacro` to pass the argument value: `<xacro:macro_name namespace="$(arg namespace)"/>`.
    3.  Updated all included macro *definitions* (e.g., `gazebo_config` in `gazebo.urdf.xacro`) to declare they accept the parameter: `<xacro:macro name="macro_name" params="namespace">`.
    4.  Updated all internal uses within the macros to access the passed argument value: `$(arg namespace)`.
    5.  Replaced `#` comments with valid XML comments `<!-- -->`.
*   **Academic Relevance:** Highlights the intricacies of macro and parameter-passing systems like XACRO. Demonstrates the importance of understanding scope and variable types (`arg` vs. `property`) in hierarchical configurations. Underscores the need for strict adherence to underlying syntax rules (XML).

### 4. Python Package Structure, `setup.py`, and Build System Interaction

Configuring the Python package build and installation using `colcon` and `setuptools` presented several challenges related to directory structure and entry point resolution.

*   **Problem Encountered:** Nodes failed to launch due to `ModuleNotFoundError: No module named 'butler_swarm'`. Subsequently, the ROS 2 launch system failed with `libexec directory '/home/butler-ws/install/butler_swarm/lib/butler_swarm' does not exist`.
*   **Investigation & Debugging Steps:**
    *   **Initial Structure:** Python code was initially placed in `src/butler_swarm/lib/`.
    *   **`setup.py` Iteration 1:** Used `package_dir={'butler_swarm': 'lib'}` to map the package name to the `lib` directory. While allowing Python imports (`import butler_swarm.agent`), this seemed incompatible with how `colcon` installed console script entry points, leading to the `ModuleNotFoundError` when the script tried to load its own package.
    *   **`setup.py` Iteration 2:** Adopted the standard ROS 2 layout by moving code to `src/butler_swarm/butler_swarm/`. Removed `package_dir` and updated `find_packages(where='.')`. Updated `entry_points` to `butler_swarm.agent:main`. This resolved the Python import issue but led to the `libexec directory ... does not exist` error.
    *   **Installation Path Inspection:** Used `ls install/butler_swarm` to observe the actual installation structure created by `colcon`. Found executables were placed in `install/butler_swarm/bin/`.
    *   **`setup.cfg` Role:** Experimented with removing `setup.cfg` which contained `[install] install_scripts=...`. Found that its presence seemed necessary to direct the script installation into the `install/PKG/lib/PKG` path that the launch system was erroneously looking for (even though `bin` is more standard for `ament_python`). Restoring `setup.cfg` resolved the `libexec` error.
    *   **Extraneous `__init__.py`:** Discovered and removed an `__init__.py` file in `src/butler_swarm/`, which may have confused `colcon` about the package root.
*   **Conclusion & Solution:** Adhering to the standard ROS 2 Python package structure (`src/pkg/pkg`) is generally recommended. However, resolving the entry point and `libexec` errors required careful configuration of `setup.py` (`packages`, `entry_points`) *and* unexpectedly relying on `setup.cfg` to influence the script installation path to match what the ROS 2 launch system was looking for in this specific environment.
*   **Academic Relevance:** Demonstrates the importance of standard package layouts in ROS 2 (`ament_python`). Illustrates the key role of `setup.py` in defining package contents and entry points for `setuptools`. Shows the complex interaction between `setup.py`, `setup.cfg`, `colcon`, and the ROS 2 launch system regarding executable paths. Highlights debugging build and installation issues by inspecting the `install` directory.

### 5. ROS 1 (`rospy`) to ROS 2 (`rclpy`) Code Refactoring

The initial codebase contained significant ROS 1 (`rospy`) dependencies requiring systematic refactoring for ROS 2 (`rclpy`) compatibility.

*   **Problem Encountered:** `ModuleNotFoundError: No module named 'rospy'` and subsequent errors related to using `rospy` APIs.
*   **Investigation & Debugging Steps:** Reviewed core agent logic (`agent.py`), behaviour implementations (`flocking.py`, `search.py`, `formation.py`), and utility classes (`communication.py`, `visualization.py`). Identified widespread use of `rospy` for node initialization, publishers, subscribers, logging, timing, and parameter handling.
*   **Conclusion & Solution:** A thorough refactoring was performed:
    *   **Node Class:** The main `SwarmAgent` class was modified to inherit from `rclpy.node.Node`.
    *   **API Replacement:** `rospy.init_node`, `rospy.Publisher`, `rospy.Subscriber`, `rospy.Rate`, `rospy.Time`, `rospy.loginfo`, etc., were replaced with their `rclpy` equivalents: `super().__init__`, `self.create_publisher`, `self.create_subscription`, `self.create_timer`, `self.get_clock`, `self.get_logger`.
    *   **Quality of Service (QoS):** QoS profiles were introduced for publishers and subscribers, essential for configuring communication reliability and history in ROS 2.
    *   **Parameter Handling:** ROS 2 parameter handling (declaration and retrieval using `self.declare_parameter`, `self.get_parameter`) was implemented.
    *   **Helper Class Interaction:** Helper classes (Behaviours, Communication, Visualization) were modified to accept the parent `Node` object in their constructors, allowing them to access the node's logger, clock, and publisher/subscriber creation methods without inheriting from `Node` themselves.
    *   **Entry Point:** Consolidated the node execution logic (`rclpy.init`, node instantiation, `rclpy.spin`, shutdown) into a `main()` function within `agent.py`, simplifying the structure.
*   **Academic Relevance:** Illustrates the fundamental API differences between ROS 1 and ROS 2. Emphasizes the architectural shift towards object-oriented node design in `rclpy`. Introduces key ROS 2 concepts like QoS and the updated parameter system. Shows practical application of dependency injection for providing node functionalities to non-node classes.

## Overall Summary

The migration from ROS 1 to ROS 2 Jazzy for the Butler Swarm Gazebo simulation was a non-trivial process involving iterative debugging across multiple layers: Python code APIs (`rospy` vs. `rclpy`), build system configuration (`colcon`, `setup.py`, `setup.cfg`), simulation model definition (URDF/XACRO namespacing, Gazebo plugin configuration), ROS-Gazebo communication (`ros_gz_bridge`, topic mapping), and crucially, the limitations of the virtualized execution environment (UTM graphics acceleration and OpenGL support). Successful migration required a systematic approach, combining code refactoring, configuration adjustments, careful inspection of build/runtime outputs (logs, topics), and research into specific tool limitations and conventions.

Ultimately, despite resolving numerous configuration and code issues related to the ROS 2 framework, build system, and Gazebo integration (achieving basic agent movement and odometry functionality), the project encountered an insurmountable obstacle with the Lidar sensor simulation. The identified graphics limitations within the UTM virtual machine environment on Apple Silicon (specifically the inability to provide OpenGL 3.3+ support required by the `gpu_lidar` sensor or compatible configurations for the `ray` sensor) prevented the acquisition of essential laser scan data. As Lidar input is fundamental for meaningful swarm behaviours like obstacle avoidance, navigation, and mapping, proceeding with the ROS 2 migration under these constraints became impractical for achieving the project's core objectives. Due to the lack of access to alternative testing environments (e.g., a native Linux machine capable of supporting Gazebo Harmonic's graphics requirements), the difficult decision was made to revert the project back to its previous ROS 1 implementation to ensure progress could be made on the primary swarm logic and algorithms, albeit without the benefits of the ROS 2 ecosystem. 