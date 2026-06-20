CXX ?= g++
PROJECT_DIR := /home/orangepi/moonxkj/mine_pedestrian

OPENCV_FLAGS := $(shell pkg-config --cflags --libs opencv4)
COMMON_FLAGS := -std=c++17 -O2 -Iinclude -I/opt/MVS/include
COMMON_LIBS := -Llib -lrknnrt -Wl,-rpath,$(PROJECT_DIR)/lib
MVS_LIBS := -L/opt/MVS/lib/aarch64 -lMvCameraControl -Wl,-rpath,/opt/MVS/lib/aarch64

.PHONY: all live image camera smoke clean

all: live image camera smoke

live:
	$(CXX) $(COMMON_FLAGS) src/mine_live_infer.cpp $(COMMON_LIBS) $(MVS_LIBS) -o bin/mine_live_infer $(OPENCV_FLAGS)

image:
	$(CXX) $(COMMON_FLAGS) src/mine_yolov8_seg.cpp $(COMMON_LIBS) -o bin/mine_yolov8_seg $(OPENCV_FLAGS)

camera:
	$(CXX) $(COMMON_FLAGS) src/mvs_capture.cpp $(MVS_LIBS) -o bin/mvs_capture $(OPENCV_FLAGS)
	$(CXX) $(COMMON_FLAGS) src/mvs_set_ip.cpp $(MVS_LIBS) -o bin/mvs_set_ip $(OPENCV_FLAGS)

smoke:
	$(CXX) -std=c++17 -O2 src/rknn_smoke.cc -Iinclude -Llib -lrknnrt -Wl,-rpath,$(PROJECT_DIR)/lib -o bin/rknn_smoke

clean:
	rm -f bin/mine_live_infer bin/mine_yolov8_seg bin/mvs_capture bin/mvs_set_ip bin/rknn_smoke
