#!/usr/bin/env bash
set -e

echo "[INFO] starting robot stack..."

# ===== 2. Zenoh bridge (client) =====
echo "[INFO] starting zenoh bridge..."
zenoh-bridge-ros2dds --rest-http-port 25000
# zenoh-bridge-ros2dds --connect tcp/127.0.0.1:7447 client&

ZENOH_PID=$!
sleep 1

# ===== 3. Foxglove bridge =====
echo "[INFO] starting foxglove bridge..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &

FOXGLOVE_PID=$!
sleep 1

echo "[OK] all processes started"

# ===== 4. graceful shutdown =====
trap "echo '[STOP] shutting down...'; kill $ZENOH_PID $FOXGLOVE_PID; exit 0" SIGINT SIGTERM

# ===== 5. block =====
wait
