cmake_minimum_required(VERSION 3.12)

project(rokae1)
#project(myplan)

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

# user program started...
# set(SOURCE_FILES src/main.cpp)
# add_executable(myplan ${SOURCE_FILES})
# target_link_libraries(myplan ${aris_LIBRARIES})

# user program started...
# set(SOURCE_FILES src/rokae/main.cpp src/rokae/rokae.cpp src/rokae/rokae.h src/rokae/plan.cpp src/rokae/plan.h)
# add_executable(rokae1 ${SOURCE_FILES})
# target_link_libraries(rokae1 ${aris_LIBRARIES})

# user program started...
set(SOURCE_FILES src/rokae/main.cpp src/rokae/rokae.cpp src/rokae/rokae.h src/rokae/plan.cpp src/rokae/plan.h src/rokae/cplan.cpp src/rokae/cplan.h)
add_executable(rokae1 ${SOURCE_FILES})
target_link_libraries(rokae1 ${aris_LIBRARIES})

# user program started...
# set(SOURCE_FILES src/test/main.cpp)
# add_executable(test ${SOURCE_FILES})
# target_link_libraries(test ${aris_LIBRARIES})
