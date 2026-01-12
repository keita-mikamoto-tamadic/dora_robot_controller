TARGET = motor_node
SRCS = main.cc
CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -I../../../cpp -I../../../vendor
LDFLAGS = -L../../../lib -ldora_node_api_c -Wl,-rpath,$(PWD)/../../../lib

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET)
