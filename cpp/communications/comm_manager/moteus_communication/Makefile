CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

PROJECT_ROOT = ../../../..
INCLUDES = -I$(PROJECT_ROOT)/cpp -I$(PROJECT_ROOT)/vendor
LDFLAGS = -L$(PROJECT_ROOT)/lib -ldora_node_api_c -Wl,-rpath,/home/jetmimic/ws/mimic-controller/lib

TARGET = moteus_communication_node

.PHONY: all clean

all: $(TARGET)

$(TARGET): main.cc rad_converter.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) main.cc rad_converter.cpp -o $@ $(LDFLAGS)

clean:
	rm -f $(TARGET) *.o
