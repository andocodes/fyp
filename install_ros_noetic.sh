#!/bin/bash

# --- ROS Noetic Installation Script (Ubuntu 20.04 ARM64) ---
# Run this script *after* vm_setup.sh to install ROS Noetic Desktop Full.
# Assumes Ubuntu 20.04 ARM64 base.

echo "--- Step 1: Setting up ROS sources.list ---"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
if [ $? -ne 0 ]; then echo "ERROR: Failed to add ROS repository."; exit 1; fi

echo "--- Step 2: Setting up ROS keys ---"
# Use apt-key method (still common for Noetic) or newer keyring method if preferred/needed
sudo apt install curl -y # ensure curl is installed
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
if [ $? -ne 0 ]; then echo "ERROR: Failed to add ROS GPG key."; exit 1; fi

# Alternative Keyring Method (uncomment below and comment out apt-key lines if desired)
# sudo apt-get update && sudo apt-get install -y curl gnupg2 lsb-release
# sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null # Note: Using ros2.list path might be fine even for ROS1 key

echo "--- Step 3: Updating package lists after adding ROS repo ---"
sudo apt-get update

echo "--- Step 4: Installing ROS Noetic Desktop Full ---"
# This step can take a while!
sudo apt-get install -y ros-noetic-desktop-full
# Robust check if installation seems successful (check a key package like roscore)
if ! dpkg -s ros-noetic-roscore > /dev/null 2>&1; then 
    echo "ERROR: ros-noetic-desktop-full installation failed (roscore not found)."; 
    exit 1; 
fi

echo "--- Step 5: Explicitly installing python3-rosdep ---"
sudo apt-get install -y python3-rosdep
if [ $? -ne 0 ]; then echo "ERROR: Failed to install python3-rosdep."; exit 1; fi

echo "--- Step 6: Initializing rosdep ---"
echo "Initializing rosdep (may show error if already initialized - this is OK)"
sudo rosdep init || echo "rosdep already initialized or failed."
rosdep update
if [ $? -ne 0 ]; then echo "WARNING: rosdep update failed. Check network connection."; fi

echo "--- Step 7: Setting up Environment Sourcing for future terminals ---"
# Add sourcing command to .bashrc for automatic environment setup in new terminals
BASHRC_LINE="source /opt/ros/noetic/setup.bash"
if ! grep -Fxq "$BASHRC_LINE" ~/.bashrc; then
    echo "$BASHRC_LINE" >> ~/.bashrc
    echo "Added ROS sourcing to ~/.bashrc"
else
    echo "ROS sourcing already in ~/.bashrc"
fi

echo "--- ROS Noetic Installation Script Finished ---"
echo "RECOMMENDED: Close and reopen your terminal or run 'source ~/.bashrc' for ROS environment changes to take effect."
echo "Next step: Copy your project code (e.g., butler-gz) into the VM."
echo "Then, navigate into the project workspace (cd ~/butler-gz) and run './run_simulation_vm.sh' which includes 'rosdep install ...' ."