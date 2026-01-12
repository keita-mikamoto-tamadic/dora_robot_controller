CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

PROJECT_ROOT = ../../..
INCLUDES = -I$(PROJECT_ROOT)/cpp -I$(PROJECT_ROOT)/vendor
LDFLAGS = -L$(PROJECT_ROOT)/lib -ldora_node_api_c -Wl,-rpath,/home/jetmimic/ws/mimic-controller/lib

TARGET = moteus_driver_node

SOURCES = main.cc can_interface.cc moteus_protocol.cc

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(SOURCES) -o $@ $(LDFLAGS)

clean:
	rm -f $(TARGET) *.o
