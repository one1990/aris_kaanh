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
# find kaanh
if(UNIX)
        add_definitions(-DUNIX)
        find_package(kaanh REQUIRED PATHS /usr/kaanh)
endif(UNIX)
IF(WIN32)
        find_package(kaanh REQUIRED PATHS C:/kaanh/kaanh-1.0)
endif(WIN32)
include_directories(${kaanh_INCLUDE_DIRS})

message(${aris_DIR})

project(kaanh_maxon)
set(SOURCE_FILES src/oneaxis_maxon.cpp src/oneaxis_maxon.h)
add_executable(kaanh_maxon ${SOURCE_FILES})
if(UNIX)
        target_link_libraries(kaanh_maxon ${kaanh_LIBRARIES} ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
        target_link_libraries(kaanh_maxon ${kaanh_LIBRARIES} ${aris_LIBRARIES})
endif(WIN32)

