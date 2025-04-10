#!/bin/bash

# --- VM Setup Script for Project Butler (Ubuntu 20.04 ARM64) ---
# Run this script inside your Ubuntu 20.04 ARM64 VM after installing the base OS.
# It installs prerequisites for Gazebo and the project, but *NOT* ROS Noetic itself.
# ROS Noetic installation on Ubuntu 20.04 ARM64 should be done via apt AFTER running this.

echo "--- Updating package lists ---"
sudo apt-get update

echo "--- Installing Git, Build Tools, CMake ---"
sudo apt-get install -y --no-install-recommends \
    git \
    build-essential \
    cmake

echo "--- Installing Python 3 and Pip ---"
sudo apt-get install -y --no-install-recommends \
    python3 \
    python3-pip

echo "--- Installing Gazebo 11 ---"
# Add OSRF repository key & setup repository
sudo apt-get update && sudo apt-get install -y wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# Install Gazebo 11 and development libraries
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    gazebo11 \
    libgazebo11-dev

echo "--- Installing Python Dependencies ---"
# Install numpy via apt for system consistency
sudo apt-get install -y --no-install-recommends python3-numpy
# If you discover other pip packages are needed later, install them here:
# Example: pip3 install --user matplotlib

echo "--- Installing ROS Build/Run Dependencies ---"
# Install common tools needed before/after ROS installation
# (These help manage ROS dependencies and build processes)
sudo apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools # Optional, provides 'catkin build' command

echo "--- Cleaning up apt cache ---"
sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

echo "--- Setup Script Finished ---"
echo "Next Step: Install ROS Noetic using the standard Ubuntu instructions:"
echo "           http://wiki.ros.org/noetic/Installation/Ubuntu"
echo "           Recommend installing 'ros-noetic-desktop-full'."
echo "           Remember to run 'sudo rosdep init' and 'rosdep update' after installing ROS."
echo "           Ensure ROS environment is sourced (e.g., 'source /opt/ros/noetic/setup.bash' in .bashrc)."