# Mimic Controller - Root Makefile
# Builds all C++ nodes

.PHONY: all clean canfd_txrx moteus_communication state_manager imu_node

# Node directories
CANFD_TXRX_DIR = cpp/communications/canfd/canfd_txrx
MOTEUS_COMM_DIR = cpp/communications/comm_manager/moteus_communication
STATE_MANAGER_DIR = cpp/state_machine/mimicv2_state_manager
IMU_NODE_DIR = cpp/sensor/imu/sony

all: canfd_txrx moteus_communication state_manager imu_node

canfd_txrx:
	$(MAKE) -C $(CANFD_TXRX_DIR)

moteus_communication:
	$(MAKE) -C $(MOTEUS_COMM_DIR)

state_manager:
	$(MAKE) -C $(STATE_MANAGER_DIR)

imu_node:
	$(MAKE) -C $(IMU_NODE_DIR)

clean:
	$(MAKE) -C $(CANFD_TXRX_DIR) clean
	$(MAKE) -C $(MOTEUS_COMM_DIR) clean
	$(MAKE) -C $(STATE_MANAGER_DIR) clean
	$(MAKE) -C $(IMU_NODE_DIR) clean
