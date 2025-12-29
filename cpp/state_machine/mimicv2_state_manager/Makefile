CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

PROJECT_ROOT = ../../..
INCLUDES = -I$(PROJECT_ROOT)/cpp -I$(PROJECT_ROOT)/vendor
LDFLAGS = -L$(PROJECT_ROOT)/lib -ldora_node_api_c -Wl,-rpath,/home/jetmimic/ws/mimic-controller/lib

TARGET = mimicv2_state_manager_node

.PHONY: all clean

all: $(TARGET)

SRCS = main.cc timing_manager.cc helpers.cc state_init.cc state_ready.cc state_stop.cc state_run.cc \
       $(PROJECT_ROOT)/cpp/control/pid_controller/pid_control.cpp

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LDFLAGS)

clean:
	rm -f $(TARGET)
