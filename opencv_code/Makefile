CC = g++
CFLAGS = -g -O2 -Wall
SRCS = stereo_depth.cpp
PROG = made

OPENCV = `pkg-config opencv --cflags --libs`
LIBS = $(OPENCV)

$(PROG):$(SRCS)
		$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)

clean:
	rm -rf made
