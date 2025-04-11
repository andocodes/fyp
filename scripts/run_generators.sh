#!/bin/bash

# Run all Project Butler result generators
echo "===== Project Butler Results Generator ====="

# Make sure scripts directory exists and scripts are executable
SCRIPT_DIR="$(dirname "$0")"
cd "$SCRIPT_DIR" || exit 1

# Make all scripts executable
chmod +x measure_performance.sh
chmod +x behavior_metrics.py
chmod +x generate_mock_results.py

# Create results directory in the root project folder
mkdir -p ../results

# Run the mock results generator (Python script)
echo "Running mock data generator..."
python3 generate_mock_results.py --root_dir ..

echo "===== Generation Complete ====="
echo "Results are available in the project root 'results' directory"
echo "Copy tables and figures into your report as needed" 