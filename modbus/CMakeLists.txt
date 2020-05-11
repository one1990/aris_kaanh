cmake_minimum_required(VERSION 3.12)

project(controlboard)

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

message(${aris_DIR} ${kaanh_DIR})
add_executable(controlboard src/main.cpp src/modbus.cpp src/modbus.h src/controlboard.cpp src/controlboard.h src/modbus_exception.h)

file(COPY src/kaanh.xml DESTINATION ./)

if(UNIX)
    target_link_libraries(controlboard ${kaanh_LIBRARIES} ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
    target_link_libraries(controlboard ${kaanh_LIBRARIES} ${aris_LIBRARIES})
endif(WIN32)

