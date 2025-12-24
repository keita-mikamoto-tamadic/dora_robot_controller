#!/bin/bash
# Simple GUI startup script for dataflow.yml

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

DATAFLOW="dataflow.yml"
DORA="/home/jetmimic/.local/bin/dora"
ROBOT_CONFIG="${ROBOT_CONFIG:-robot_config/mimic_v2.json}"

cleanup() {
    echo ""
    echo "[startup] Stopping GUI..."
    kill $GUI_PID 2>/dev/null || true
    sleep 1
    echo "[startup] Stopping dataflow..."
    $DORA stop --name "$(basename "$DATAFLOW" .yml)" 2>/dev/null || true
    sleep 1
    echo "[startup] Destroying dora daemon..."
    $DORA destroy 2>/dev/null || true
    echo "[startup] Done"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "=== Simple Motor Test ==="
echo "  Dataflow: $DATAFLOW"
echo "  Robot Config: $ROBOT_CONFIG"
echo "  Press Ctrl+C to stop"
echo "========================="

# Set RT capability on nodes (requires sudo password)
CANFD_NODE="cpp/communications/canfd/canfd_txrx/canfd_txrx_node"
MOTEUS_NODE="cpp/communications/comm_manager/moteus_communication/moteus_communication_node"
echo "[startup] Setting RT capability (sudo required)..."
sudo setcap cap_sys_nice+ep "$CANFD_NODE"
sudo setcap cap_sys_nice+ep "$MOTEUS_NODE"

# Start dora daemon
echo "[startup] Starting dora daemon..."
$DORA up

# Start dataflow
echo "[startup] Starting dataflow..."
$DORA start "$DATAFLOW"

# Wait for dataflow to initialize
sleep 2

# Start simple GUI
echo "[startup] Starting Simple GUI..."
ROBOT_CONFIG="$ROBOT_CONFIG" python3 python/gui/simple_motor_controller/simple_gui_node.py &
GUI_PID=$!

# Wait forever until Ctrl+C
echo "[startup] Running... (Ctrl+C to stop)"
while true; do
    sleep 1
done
