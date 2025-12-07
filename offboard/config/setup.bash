# Bash setup generated at 2025-11-19 05:57:22.120623+00:00
# Core ROS setup
source /opt/ros/jazzy/setup.bash
# ROS configuration
export ROS_DOMAIN_ID="0"
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
export ROS_DISCOVERY_SERVER="10.10.0.7:11811;10.10.10.42:11811;10.10.10.37:11811;127.0.0.1:11811;"
if [ -t 0 ]; then
  export ROS_SUPER_CLIENT="True"
else
  unset ROS_SUPER_CLIENT
fi
export ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET"
unset ROS_STATIC_PEERS
