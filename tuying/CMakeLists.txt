cmake_minimum_required(VERSION 3.12)

project(tuying)

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
# find kaanh
if(UNIX)
        add_definitions(-DUNIX)
        find_package(kaanh REQUIRED PATHS /usr/kaanh)
endif(UNIX)
IF(WIN32)
        find_package(kaanh REQUIRED PATHS C:/kaanh/kaanh-1.0)
endif(WIN32)
include_directories(${kaanh_INCLUDE_DIRS})

if(UNIX)
        include_directories(${aris_INCLUDE_DIRS} /usr/local/include/dynamixel_sdk)
        link_directories(/home/kaanh/Desktop/Dynamixel/Dynamixel/c++/build/linux64)
endif(UNIX)
IF(WIN32)
        include_directories(${aris_INCLUDE_DIRS} D:/1working/tuying/Dynamixel/Dynamixel/c++/include/dynamixel_sdk)
endif(WIN32)

message(${aris_DIR})

# user program started...
set(SOURCE_FILES src/main.cpp src/tuying.cpp src/tuying.h src/dxl.cpp src/dxl.h src/json.hpp)
add_executable(tuying ${SOURCE_FILES})

if(UNIX)
    target_link_libraries(tuying ${kaanh_LIBRARIES} ${aris_LIBRARIES} atomic libdxl_x64_cpp.so)
endif(UNIX)
IF(WIN32)
	target_link_libraries(tuying ${kaanh_LIBRARIES} ${aris_LIBRARIES} D:/1working/tuying/Dynamixel/Dynamixel/c++/build/win64/output/dxl_x64_cpp.lib)
endif(WIN32)
