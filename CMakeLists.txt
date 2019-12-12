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

message(${aris_DIR})
add_library(kaanh_lib STATIC src/kaanh.cpp src/kaanh.h src/kaanhconfig.cpp src/kaanhconfig.h src/planfuns.cpp src/planfuns.h src/sixdistalfc.cpp src/sixdistalfc.h src/jointdynamics.cpp src/jointdynamics.h src/sixdistaldynamics.cpp src/sixdistaldynamics.h)
add_executable(kaanh src/main.cpp)
target_link_libraries(kaanh kaanh_lib)

if(UNIX)
	target_link_libraries(kaanh ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
	target_link_libraries(kaanh ${aris_LIBRARIES})
endif(WIN32)

project(kaanh_jmc)
set(SOURCE_FILES src/oneaxis_jmc.cpp src/oneaxis_jmc.h)
add_executable(kaanh_jmc ${SOURCE_FILES})
if(UNIX)
	target_link_libraries(kaanh_jmc ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
	target_link_libraries(kaanh_jmc ${aris_LIBRARIES})
endif(WIN32)
