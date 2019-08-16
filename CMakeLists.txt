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


add_library(kaanh_lib STATIC src/kaanh.cpp src/iir.cpp src/forcecontrol.cpp src/kaanh.h src/iir.h src/forcecontrol.h src/cplan.cpp src/cplan.h src/robotconfig.cpp src/robotconfig.h src/sixdistalfc.cpp src/sixdistalfc.h src/sixdistaldynamics.cpp src/sixdistaldynamics.h src/jointdynamics.cpp src/jointdynamics.h src/jointfc.cpp src/jointfc.h src/sevenjointdynamics.cpp src/sevenjointdynamics.h src/sevenjointfc.cpp src/sevenjointfc.h src/tempvariables.h src/move_series.cpp src/move_series.h src/kinematics.cpp src/kinematics.h)


# user program started...
#set(SOURCE_FILES src/main.cpp)
#add_executable(kaanh ${SOURCE_FILES})
add_executable(kaanh src/main.cpp)
target_link_libraries(kaanh kaanh_lib)


if(UNIX)
	target_link_libraries(kaanh ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
	target_link_libraries(kaanh ${aris_LIBRARIES})
endif(WIN32)
