CXX = g++
CXXFLAGS = -std=c++17 -Wall -O2
LDFLAGS = 

SRC_DIR = kpp_robotik
SRCS = \
	$(SRC_DIR)/Graph.cpp \
	$(SRC_DIR)/KPPRobotik.cpp \
	$(SRC_DIR)/main.cpp

OBJS = $(SRCS:.cpp=.o)
TARGET = kpp_robotik/KPPRobotik

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

$(SRC_DIR)/%.o: $(SRC_DIR)/%.cpp $(SRC_DIR)/%.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(SRC_DIR)/*.o $(TARGET)

.PHONY: all clean
