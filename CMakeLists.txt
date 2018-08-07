cmake_minimum_required(VERSION 3.6)
project(vdec)

#rtsp://admin:admin123@192.168.4.1/ch1/h264/av_stream
#rtsp://admin:admin@192.168.19.101:554/cam/realmonitor?channel=1&subtype=0

set(CMAKE_C_COMPILER aarch64-himix100-linux-gcc)
set(CMAKE_CXX_COMPILER aarch64-himix100-linux-g++)

set(CMAKE_CXX_FLAGS_DEBUG   "$ENV{CXXFLAGS} -O0 -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -fpermissive")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -DHI3559av100 -DHI_XXXX -DISP_V2 -fmessage-length=0 -MMD -MP -MF")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -DHI_ACODEC_TYPE_INNER -DHI_ACODEC_TYPE_HDMI -fno-aggressive-loop-optimizations -ffunction-sections -fdata-sections -O3 -fstack-protector -fPIC")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -Wall -Wno-error=implicit-function-declaration -DUSER_BIT_64 -DKERNEL_BIT_64 -DENABLE_JPEGEDCF -Wno-date-time")

add_definitions(-D_FILE_OFFSET_BITS=64)
add_definitions(-DVER_X=1)
add_definitions(-DVER_Y=0)
add_definitions(-DVER_P=0)
add_definitions(-DVER_B=10)

#add_definitions(-DSAVE)

add_definitions(-DSENSOR0_TYPE=SONY_IMX477_MIPI_8M_30FPS_12BIT)
add_definitions(-DSENSOR1_TYPE=SONY_IMX477_MIPI_8M_30FPS_12BIT)
add_definitions(-DSENSOR2_TYPE=SONY_IMX477_MIPI_8M_30FPS_12BIT)
add_definitions(-DSENSOR3_TYPE=SONY_IMX477_MIPI_8M_30FPS_12BIT)
add_definitions(-DSENSOR4_TYPE=SONY_IMX477_MIPI_8M_30FPS_12BIT)
add_definitions(-DSENSOR5_TYPE=SONY_IMX477_MIPI_8M_30FPS_12BIT)
add_definitions(-DSENSOR6_TYPE=SONY_IMX477_MIPI_8M_30FPS_12BIT)
add_definitions(-DSENSOR7_TYPE=SONY_IMX477_MIPI_8M_30FPS_12BIT)

include_directories(include/hisi)
include_directories(include/opencv)
include_directories(include/ffmpeg)
include_directories(include/deepglint)

link_directories(lib/hisi)
link_directories(lib/ffmpeg)
link_directories(lib/opencv)

add_subdirectory(src/commom)
add_subdirectory(src/deepglint)

set(HISI_LIB_PATH lib/hisi)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

add_executable(NeverMore NeverMore.cpp)

target_link_libraries(NeverMore deepglint hisi_common libnnie.a libmpi.a libive.a libhdmi.a libVoiceEngine.a libupvqe.a libdnvqe.a libsecurec.a
        libavdevice.a libavformat.a libavcodec.a libavfilter.a libavutil.a libswresample.a libopencv_core.a libopencv_highgui.a
        libzlib.a -lpthread -lm -ldl -lstdc++)