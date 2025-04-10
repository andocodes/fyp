# Project Butler: ROS/Gazebo Swarm Intelligence Simulation

## Overview

Project Butler is a simulation environment designed to explore and demonstrate concepts of swarm intelligence using the Robot Operating System (ROS) Noetic and the Gazebo simulator. It focuses on emergent behaviours arising from local interactions between simple agents, collaborative task performance, and dynamic pattern formation.

This project implements a swarm of differential drive robots capable of exhibiting behaviours like flocking, collaborative search, and various geometric formations within a simulated environment.

## Code Structure

The core logic resides within the `src/` directory, organized as a single ROS package (`butler_swarm`):

*   **`src/lib/`**: Contains the primary Python source code.
    *   `agent.py`: Defines the `SwarmAgent` class, handling individual agent logic, sensing, communication, and behaviour switching.
    *   `swarm_controller.py`: A central node (can be adapted for decentralized control) to potentially coordinate high-level swarm behaviour or parameters.
    *   `spawn_agents.py`: Script responsible for spawning the specified number of agent models into the Gazebo simulation.
    *   `lib/behaviours/`: Python modules implementing specific swarm behaviours.
        *   `flocking.py`: Implements Reynolds' Boids rules (separation, alignment, cohesion).
        *   `search.py`: Implements a collaborative environment mapping/exploration strategy.
        *   `formation.py`: Implements dynamic formation control (line, circle, grid, etc.).
    *   `lib/utils/`: Utility modules.
        *   `communication.py`: Handles ROS topic-based communication between agents.
        *   `visualization.py`: Helper classes for publishing visualization markers to RViz.
*   **`src/launch/`**: Contains ROS launch files to start the simulation components.
    *   `butler_swarm.launch`: The main launch file to start Gazebo, RViz, the swarm controller, and spawn agents.
    *   `world.launch`: (If used separately) Launches the Gazebo world.
    *   `spawn_agent.launch`: (Likely unused directly, logic is in `spawn_agents.py`) Potentially for spawning single agents.
*   **`src/models/`**: Contains the URDF/Xacro files defining the agent robot model for Gazebo.
    *   `butler_agent.urdf.xacro`: The description file for the simulated robot.
*   **`src/worlds/`**: Contains Gazebo world definition files.
    *   `empty_arena.world`: A simple world file used for the simulation.
*   **`src/config/`**: (Optional, create if needed) Can store configuration files, such as saved RViz configurations (`swarm.rviz`).
*   **`package.xml`**: Defines ROS package information and dependencies.
*   **`CMakeLists.txt`**: CMake build file required by the ROS Catkin build system.

## Root Directory Scripts

*   **`vm_setup.sh`**: Installs system prerequisites (build tools, Python, Gazebo, ROS tooling like `rosdep`) needed before installing ROS itself and building the project. **Does NOT install ROS Noetic.**
*   **`install_ros_noetic.sh`**: Installs ROS Noetic Desktop Full on Ubuntu 20.04 ARM64 using `apt`, including setting up repositories, keys, environment sourcing (`.bashrc`), and `rosdep` initialization. Should be run after `vm_setup.sh`.
*   **`run_simulation_vm.sh`**: Installs project-specific ROS dependencies (`rosdep install`), builds the Catkin workspace (`catkin_make`), sources the environment, and launches the main simulation (`roslaunch`). Should be run after `install_ros_noetic.sh` and after reopening the terminal.

## Setup and Installation

This project requires **Ubuntu 20.04 (Focal Fossa)** and ROS Noetic.

**Scenario A: Running on macOS (Apple Silicon M1/M2/M3) via UTM Virtual Machine**

Using a dedicated Linux VM avoids Docker graphics emulation issues.

1.  **Install UTM:** Download and install UTM ([https://mac.getutm.app/](https://mac.getutm.app/)).
2.  **Download Ubuntu ISO:** Get the **Ubuntu 20.04.6 (Focal Fossa) ARM64 Server** ISO ([https://releases.ubuntu.com/focal/](https://releases.ubuntu.com/focal/)). File: `ubuntu-20.04.6-live-server-arm64.iso`.
3.  **Create VM in UTM:**
    *   Virtualize -> Linux -> Select downloaded ISO.
    *   Hardware: 4GB+ Memory, desired CPU cores, **Enable Hardware OpenGL acceleration**.
    *   Storage: 30GB+ disk space.
    *   Shared Directory: Configure share pointing to your local `butler-gz` folder (Tag name e.g., `share`).
    *   Save and name the VM.
4.  **Install Ubuntu Server:** Start VM, follow installation prompts. Create user account.
5.  **Install Desktop Environment:** After server install & reboot:
    *   Log in to command line.
    *   `sudo apt update && sudo apt upgrade -y`
    *   `sudo apt install -y ubuntu-desktop` (or another DE like `xfce4 xfce4-goodies`)
    *   `sudo reboot`
    *   Log in to the graphical desktop.
6.  **Mount Shared Directory:** Open a terminal in the VM.
    ```bash
    sudo mkdir /mnt/host_share
    sudo mount -t 9p -o trans=virtio share /mnt/host_share # Use your share tag name
    ```
7.  **Copy Project & Scripts:** Copy the *entire* `butler-gz` project folder from `/mnt/host_share/butler-gz` to your VM's home directory (`~/butler-gz`). Ensure `vm_setup.sh`, `install_ros_noetic.sh`, and `run_simulation_vm.sh` are present in `~/butler-gz`.
8.  **Run Setup Scripts:** Open a terminal in the VM:
    ```bash
    cd ~/butler-gz
    chmod +x vm_setup.sh install_ros_noetic.sh run_simulation_vm.sh
    # Run prerequisite installer
    ./vm_setup.sh
    # Run ROS Noetic installer
    ./install_ros_noetic.sh 
    ```
9.  **IMPORTANT: Restart Terminal:** Close the current terminal and open a **new** one. This ensures the ROS environment variables added to `.bashrc` by `install_ros_noetic.sh` are loaded.
10. **Run Simulation:** In the new terminal:
    ```bash
    cd ~/butler-gz
    ./run_simulation_vm.sh
    ```

**Scenario B: Running on a Native Linux Machine (Ubuntu 20.04 AMD64/ARM64)**

1.  **Clone Repository:**
    ```bash
    git clone <your-repo-url> ~/butler-gz
    cd ~/butler-gz
    ```
2.  **Run Setup Scripts:**
    ```bash
    chmod +x vm_setup.sh install_ros_noetic.sh run_simulation_vm.sh
    # Run prerequisite installer
    ./vm_setup.sh
    # Run ROS Noetic installer
    ./install_ros_noetic.sh
    ```
3.  **IMPORTANT: Restart Terminal:** Close the current terminal and open a **new** one to load the ROS environment.
4.  **Run Simulation:** In the new terminal:
    ```bash
    cd ~/butler-gz 
    ./run_simulation_vm.sh
    ```

## Running the Simulation (Post-Setup)

Once setup is complete for either scenario:

1.  **Open a Terminal** (ensure ROS environment is sourced).
2.  **Navigate to Project Directory:** `cd ~/butler-gz`
3.  **Execute:** `./run_simulation_vm.sh`

This script now handles project dependency installation (`rosdep`), building (`catkin_make`), and launching (`roslaunch`).

## RViz Configuration

1.  While the simulation is running, configure RViz to display relevant topics (e.g., `/tf`, `/agent_*/odom`, `/agent_*/scan`, `/visualization/*`).
2.  Once you have a desired layout, save the configuration: RViz Menu -> File -> Save Config As...
3.  Save the file as `swarm.rviz` inside a *new* directory `~/butler-gz/config/`.
4.  Edit `~/butler-gz/src/launch/butler_swarm.launch`. Find the RViz node definition and change the `args` attribute back to load your config:
    ```xml
    <!-- Change from: -->
    <!-- <node name="rviz" pkg="rviz" type="rviz" args="" output="screen"/> -->
    <!-- To: -->
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find butler_swarm)/config/swarm.rviz" output="screen"/>
    ```
5.  The next time you run `./run_simulation_vm.sh`, RViz should load your saved configuration.
