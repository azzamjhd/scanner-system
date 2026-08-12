#!/usr/bin/env bash
# Start rosbridge websocket + rosapi with a clean Python 3.12 path
# (avoids conflict with Hermes venv's Python 3.11 numpy)
#
# rosbridge_websocket — WebSocket gateway (port 9090) for the tablet UI
# rosapi            — ROS graph introspection services (/rosapi/nodes, /rosapi/topics…)
#                     used by the UI to detect real ROS 2 connectivity
source /opt/ros/jazzy/setup.bash
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages

# rosbridge WebSocket gateway (foreground; keeps the script alive)
exec /usr/bin/python3 /opt/ros/jazzy/lib/rosbridge_server/rosbridge_websocket &
ROSBRIDGE_PID=$!

# rosapi graph introspection node (background)
/usr/bin/python3 /opt/ros/jazzy/lib/rosapi/rosapi_node &
ROSAPI_PID=$!

# Keep both running; forward SIGTERM/SIGINT to children
trap 'kill $ROSBRIDGE_PID $ROSAPI_PID 2>/dev/null' EXIT INT TERM
wait $ROSBRIDGE_PID
