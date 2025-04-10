#!/bin/bash

# Script to switch swarm behaviour by publishing to ROS topics

# Check if an argument is provided
if [ -z "$1" ]; then
  echo "Usage: ./behaviour_switch.sh <behaviour_name>"
  echo "Available behaviours: idle, search, flock, explore, roundtable, line, grid"
  exit 1
fi

BEHAVIOUR=$1
JSON_BEHAVIOUR=""
JSON_FORMATION=""

case $BEHAVIOUR in
  idle)
    JSON_BEHAVIOUR='{"behaviour":"idle"}'
    ;;
  search)
    JSON_BEHAVIOUR='{"behaviour":"search"}'
    ;;
  flock)
    JSON_BEHAVIOUR='{"behaviour":"flocking"}'
    ;;
  explore)
    JSON_BEHAVIOUR='{"behaviour":"explore"}'
    ;;
  roundtable)
    JSON_FORMATION='{"formation":"roundtable"}'
    JSON_BEHAVIOUR='{"behaviour":"formation"}'
    ;;
  line)
    JSON_FORMATION='{"formation":"line"}'
    JSON_BEHAVIOUR='{"behaviour":"formation"}'
    ;;
  grid)
    JSON_FORMATION='{"formation":"grid"}'
    JSON_BEHAVIOUR='{"behaviour":"formation"}'
    ;;
  *)
    echo "Error: Unknown behaviour '$BEHAVIOUR'"
    echo "Available behaviours: idle, search, flock, explore, roundtable, line, grid"
    exit 1
    ;;
esac

# Publish formation command if needed
if [ -n "$JSON_FORMATION" ]; then
  echo "Setting formation to $BEHAVIOUR..."
  rostopic pub /swarm/formation std_msgs/String "data: '$JSON_FORMATION'" -1
  # Add a small delay to allow agents to process the formation command
  sleep 0.5 
fi

# Publish behaviour command
echo "Switching behaviour to $BEHAVIOUR..."
rostopic pub /swarm/behaviour std_msgs/String "data: '$JSON_BEHAVIOUR'" -1

echo "Command sent." 