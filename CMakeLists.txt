cmake_minimum_required(VERSION 3.12)

project(test_for_envision)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

# find aris
find_package(aris REQUIRED HINTS /usr/aris/)
include_directories(${aris_INCLUDE_DIRS})

# user program started...
set(SOURCE_FILES src/main.cpp)
add_executable(test_for_envision ${SOURCE_FILES})
target_link_libraries(test_for_envision ${aris_LIBRARIES})
