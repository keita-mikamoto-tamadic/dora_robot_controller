CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -pthread

PROJECT_ROOT = ../../../..
INCLUDES = -I$(PROJECT_ROOT)/cpp -I. -I/usr/include/eigen3
LDFLAGS = -L$(PROJECT_ROOT)/lib -ldora_node_api_c -Wl,-rpath,/home/jetmimic/ws/mimic-controller/lib

TARGET = imu_node

.PHONY: all clean

all: $(TARGET)

$(TARGET): imu_node.cc spresense_imu.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) imu_node.cc spresense_imu.cc -o $@ $(LDFLAGS)

clean:
	rm -f $(TARGET) *.o
