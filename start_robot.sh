#!/bin/bash
# Robot Control startup script (with state manager)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

DATAFLOW="dataflow_robot.yml"
DORA="/home/jetmimic/.local/bin/dora"

cleanup() {
    echo ""
    echo "[startup] Stopping dataflow..."
    $DORA stop --name "$(basename "$DATAFLOW" .yml)" 2>/dev/null || true
    sleep 1
    echo "[startup] Destroying dora daemon..."
    $DORA destroy 2>/dev/null || true
    echo "[startup] Done"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "=== Robot Control (State Manager) ==="
echo "  Dataflow: $DATAFLOW"
echo "  Press Ctrl+C to stop"
echo "======================================"

# Set RT capability on nodes (requires sudo password)
CANFD_NODE="cpp/communications/canfd/canfd_txrx/canfd_txrx_node"
MOTEUS_NODE="cpp/communications/comm_manager/moteus_communication/moteus_communication_node"
STATE_NODE="cpp/state_machine/mimicv2_state_manager/mimicv2_state_manager_node"
echo "[startup] Setting RT capability (sudo required)..."
sudo setcap cap_sys_nice+ep "$CANFD_NODE"
sudo setcap cap_sys_nice+ep "$MOTEUS_NODE"
sudo setcap cap_sys_nice+ep "$STATE_NODE"

# Start dora daemon (as normal user, not root)
echo "[startup] Starting dora daemon..."
$DORA up

# Start dataflow
echo "[startup] Starting dataflow..."
$DORA start "$DATAFLOW"

# Wait forever until Ctrl+C
echo "[startup] Running... (Ctrl+C to stop)"
while true; do
    sleep 1
done
