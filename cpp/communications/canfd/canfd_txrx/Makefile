CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

PROJECT_ROOT = ../../../..
INCLUDES = -I$(PROJECT_ROOT)/cpp
LDFLAGS = -L$(PROJECT_ROOT)/lib -ldora_node_api_c -Wl,-rpath,/home/jetmimic/ws/mimic-controller/lib

TARGET = canfd_txrx_node
SRCS = main.cc can_if.cc

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRCS) can_if.hpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(SRCS) -o $@ $(LDFLAGS)

clean:
	rm -f $(TARGET)
