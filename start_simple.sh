#!/bin/bash
cd "$(dirname "$0")"

cleanup() {
    echo "Stopping..."
    dora stop 2>/dev/null
    dora destroy 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "=== Simple Robot Control ==="

# RT capability
sudo setcap cap_sys_nice+ep cpp/simple/motor_node/motor_node

dora up
dora start dataflow_simple.yml

echo "Running... Ctrl+C to stop"
while true; do sleep 1; done
