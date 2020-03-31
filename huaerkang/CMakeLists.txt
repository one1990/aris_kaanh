cmake_minimum_required(VERSION 3.12)

project(kaanh)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

# find aris
if(UNIX)
	add_definitions(-DUNIX)
        find_package(aris REQUIRED PATHS /usr/aris)
endif(UNIX)
IF(WIN32)
        find_package(aris REQUIRED PATHS C:/aris/aris-1.5.0)
endif(WIN32)
include_directories(${aris_INCLUDE_DIRS})

if(UNIX)
	include_directories(usr/include/kaanh)
endif(UNIX)

if(WIN32)
	link_directories(D:/1working/build_aris_kaanh/Debug)
endif(WIN32)

message(${aris_DIR})
add_executable(kaanh src/main.cpp src/kaanh.h src/config.cpp src/config.h src/kaanhconfig.h)

if(UNIX)
	target_link_libraries(kaanh libkaanh_lib.a)
endif(UNIX)

IF(WIN32)
	target_link_libraries(kaanh kaanh_lib)
endif(WIN32)

if(UNIX)
    target_link_libraries(kaanh ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
	target_link_libraries(kaanh ${aris_LIBRARIES})
endif(WIN32)
