# This makefile is not to be used for regular building of this code, for that please build using cmake
# This Makefile exists in case you wish to build these files into a traditional "lib" "include" "src" directory structure for linking to other applications.
CXX = g++
CXXFLAGS = -g -c -O2 -Wall
INCL = -I./include/ -I/usr/include/eigen3/ -I/home/peter/REU_Workspace/ORB_SLAM2/ -I/home/peter/REU_Workspace/Pangolin/include -I/home/peter/REU_Workspace/Pangolin/build/src/include -I/home/peter/REU_Workspace/ORB_SLAM2/include/

OPENCV = `pkg-config opencv --cflags --libs`
LIBS = $(OPENCV)

SOURCES=$(wildcard src/*.cpp)
OBJECTS=$(patsubst src/%.cpp, lib/%.o, $(SOURCES))

all: $(OBJECTS)

$(OBJECTS): lib/%.o : src/%.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(INCL) $(LIBS)

clean :
	rm -rf made