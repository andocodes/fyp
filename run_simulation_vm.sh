#!/bin/bash

# --- VM Simulation Launch Script for Project Butler ---
# Run this script inside your Ubuntu 20.04 ARM64 VM.
# Assumes:
# 1. ROS Noetic is installed via apt and sourced (e.g., in .bashrc).
# 2. Project code is located in the same directory as this script (or specified by PROJECT_DIR).
# 3. Base dependencies installed via vm_setup.sh and ROS installed via install_ros_noetic.sh.

# Get the directory where this script is located
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PROJECT_DIR=${SCRIPT_DIR} # Assume script is in the project root

echo "--- Navigating to project directory: ${PROJECT_DIR} ---"
cd "${PROJECT_DIR}" || exit 1

echo "--- Installing project-specific ROS dependencies --- "
# This command checks package.xml files in src/ and installs needed dependencies.
# It might ask for sudo password if system packages need installing.
rosdep install --from-paths src --ignore-src -r -y

# Check if rosdep install was successful before proceeding
if [ $? -ne 0 ]; then
    echo "ERROR: rosdep install failed. Please check dependency errors."
    exit 1
fi

echo "--- Cleaning previous build artifacts (if any) ---"
# Keep logs directory for debugging failed nodes like gzserver
# rm -rf build/ devel/ logs/
rm -rf build/ devel/

echo "--- Building the workspace with catkin_make ---"
catkin_make

# Check if build was successful before sourcing and launching
if [ $? -ne 0 ]; then
    echo "ERROR: catkin_make failed. Please check build errors."
    exit 1
fi

echo "--- Sourcing the workspace ---"
# Source the setup file for the current shell session
# Note: Ensure ROS base environment is already sourced via ~/.bashrc
source devel/setup.bash

echo "--- Launching the simulation ---"
# Use the main launch file for the project and filter TF_REPEATED_DATA warnings from stderr
roslaunch butler_swarm butler_swarm.launch 2> >(grep -v TF_REPEATED_DATA >&2)

echo "--- Simulation finished or terminated ---" 