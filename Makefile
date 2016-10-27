BUILD_DIR = $(shell pwd)/build

######## LIBRAIRIES
CXX = g++

INCLUDE_OpenCV = -I/usr/local/include/opencv
OpenCV_DIR = -L/hamidb/software/opencv/build
LIBS_OpenCV = $(OpenCV_DIR)/lib -lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_xfeatures2d
LIBS =  -L/usr/local/lib -lpthread $(LIBS_OpenCV)

#CXXFLAGS += -g -O3 -std=c++11 -Wall  $(INCLUDE_OpenCV) -I.
#LDFLAGS += $(LIBS)

# or un-comment the following two lines
#-DPTHREAD_V1 
EXTRA_FLAGS = -DPTHREAD_V1 -DNUM_THREADS=4 -DSHOW_RESULT=false
CXXFLAGS += -g -O3 -std=c++11 -Wall $(shell pkg-config --cflags opencv) -I. $(EXTRA_FLAGS)
LDFLAGS += $(shell pkg-config --libs opencv) $(LIBS)

TARGET1 = viewSynth
SRCS1 = main.cpp warping.cpp
HEADERS = warping.h yuv.h log.h
OBJS1 = $(patsubst %.cpp, $(BUILD_DIR)/%.o, $(SRCS1))

.PHONY: all orb clean

all: orb

orb: $(BUILD_DIR)/$(TARGET1)

$(BUILD_DIR)/$(TARGET1): $(OBJS1)
		$(CXX) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/%.o : %.cpp $(HEADERS)
		@mkdir -p $(BUILD_DIR)
			$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
		rm -rf $(BUILD_DIR)/$(TARGET1) $(OBJS1)
