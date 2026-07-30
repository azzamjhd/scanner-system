#!/usr/bin/env bash
# Start rosbridge websocket with a clean Python 3.12 path
# (avoids conflict with Hermes venv's Python 3.11 numpy)
source /opt/ros/jazzy/setup.bash
exec env PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages \
  /usr/bin/python3 /opt/ros/jazzy/lib/rosbridge_server/rosbridge_websocket
