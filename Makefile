# Mimic Controller - Root Makefile
# Builds all C++ nodes

.PHONY: all clean canfd_txrx moteus_communication

# Node directories
CANFD_TXRX_DIR = cpp/communications/canfd/canfd_txrx
MOTEUS_COMM_DIR = cpp/communications/comm_manager/moteus_communication

all: canfd_txrx moteus_communication

canfd_txrx:
	$(MAKE) -C $(CANFD_TXRX_DIR)

moteus_communication:
	$(MAKE) -C $(MOTEUS_COMM_DIR)

clean:
	$(MAKE) -C $(CANFD_TXRX_DIR) clean
	$(MAKE) -C $(MOTEUS_COMM_DIR) clean
