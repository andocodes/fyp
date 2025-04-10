cmake_minimum_required(VERSION 3.0.2)
project(butler_swarm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  gazebo_ros
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES butler_swarm
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs visualization_msgs gazebo_ros
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  nodes/agent_node.py
  nodes/swarm_controller.py
  nodes/spawn_agents.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Note: The catkin_python_setup() call handles installing the library code from lib/
# No need to list lib/agent.py etc. here anymore unless they are ALSO meant
# to be directly executable scripts (which agent.py is not anymore).