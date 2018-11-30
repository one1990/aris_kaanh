cmake_minimum_required(VERSION 3.12)

project(kaanh)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

# find aris
find_package(aris REQUIRED HINTS /usr/aris/)
include_directories(${aris_INCLUDE_DIRS})

# user program started...
set(SOURCE_FILES src/main.cpp src/rokae.cpp src/iir.cpp)
add_executable(kaanh ${SOURCE_FILES})
target_link_libraries(kaanh ${aris_LIBRARIES})
