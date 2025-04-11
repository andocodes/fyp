#!/bin/bash

# Performance measurement script for Project Butler
# Creates data files with real measurements for simulation environment metrics

echo "=== Performance Metrics Measurement Tool ==="
echo "This script will measure performance metrics for the Project Butler simulation"

# Create output directory
mkdir -p results/performance

# Function to measure CPU, RAM, and other metrics during simulation
measure_metrics() {
  num_agents=$1
  duration=$2
  output_file="results/performance/metrics_${num_agents}_agents.txt"
  
  echo "Starting measurement for ${num_agents} agents (${duration} seconds)..."
  
  # Start the simulation in background
  roslaunch butler_swarm butler_swarm.launch num_agents:=${num_agents} &
  sim_pid=$!
  
  # Wait for simulation to fully start
  sleep 10
  
  echo "Measuring metrics for ${duration} seconds..."
  
  # Initialize metrics file
  echo "# Butler Swarm Performance Metrics - ${num_agents} Agents" > "$output_file"
  echo "# Timestamp,CPU(%),RAM(MB),Real-time Factor,Sensor Rate(Hz)" >> "$output_file"
  
  # Measure for specified duration
  start_time=$(date +%s)
  while [ $(($(date +%s) - start_time)) -lt "$duration" ]; do
    # CPU for Gazebo processes
    cpu=$(top -b -n 1 | grep -i gazebo | awk '{sum+=$9} END {print sum}')
    
    # RAM usage in MB
    ram=$(ps aux | grep -i gazebo | awk '{sum+=$6} END {print sum/1024}')
    
    # Get real-time factor by comparing sim time to real time
    sim_time=$(rostopic echo -n 1 /clock | grep -A 1 secs | tail -n 1 | awk '{print $2}')
    real_time=$(date +%s)
    rtf=$(echo "scale=2; ($sim_time)/($real_time - $start_time)" | bc)
    
    # Get sensor update rate 
    scan_rate=$(rostopic hz /agent_0/scan -w 2 2>&1 | grep average | awk '{print $3}')
    
    # Record measurements
    echo "$(date +%s),$cpu,$ram,$rtf,$scan_rate" >> "$output_file"
    
    # Wait before next measurement
    sleep 2
  done
  
  # Stop the simulation
  kill $sim_pid
  sleep 5
  
  echo "Measurements complete for ${num_agents} agents"
  echo "Results saved to: $output_file"
}

# Run measurements for different numbers of agents
measure_metrics 1 60
measure_metrics 3 60
measure_metrics 5 60

echo "=== All measurements complete ==="
echo "Results are stored in the results/performance directory" 