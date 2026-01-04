#!/usr/bin/env bash
set -e

echo "[INFO] starting robot stack..."

# ===== 3. Foxglove bridge =====
echo "[INFO] starting foxglove bridge..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &

sleep 1

# ===== 2. Zenoh bridge (client) =====
echo "[INFO] starting zenoh bridge..."
zenoh-bridge-ros2dds --rest-http-port 25000 --namespace /rt --no-multicast-scouting

