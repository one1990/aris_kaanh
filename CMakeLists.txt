cmake_minimum_required(VERSION 3.12)

project(aris_kaanh)

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_CXX_STANDARD 17)

# set install directories
if(UNIX)
	add_definitions(-DUNIX)
	set(RELY_LINK_LIB pthread stdc++fs)#pthread is needed for concurrency
	
	# rely on etherlab
	set(RELY_INCL_DIR "/opt/etherlab/include" ${RELY_INCL_DIR})
	set(RELY_LINK_DIR "/opt/etherlab/lib" ${RELY_LINK_DIR})
	set(RELY_LINK_LIB ethercat_rtdm ${RELY_LINK_LIB})
	
	# rely on xenomai
	message("using xenomai")
	
	add_definitions(-DUSE_XENOMAI)
	set(RELY_LINK_DIR "/usr/xenomai/lib" ${RELY_LINK_DIR})
	set(XENOMAI_ROOT_DIR "/usr/xenomai")
	
	# copy this file to you project/config
	set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config)
	find_package(Xenomai REQUIRED)
	include_directories(${XENOMAI_INCLUDE_DIRS})
	set(RELY_LINK_LIB ${XENOMAI_LIBRARIES} ${RELY_LINK_LIB})

	# rely on aris
	set(RELY_INCL_DIR "/usr/aris/include" ${RELY_INCL_DIR})
	set(RELY_LINK_DIR "/usr/aris/lib/release" ${RELY_LINK_DIR})
	set(RELY_LINK_LIB aris_robot aris_server aris_sensor aris_plan aris_dynamic aris_control aris_core ${RELY_LINK_LIB})
endif(UNIX)


if(WIN32)
	set(RELY_LINK_LIB "ws2_32")#ws2_32 is needed for socket
	set(RELY_INCL_DIR "C:/aris/aris-1.5.0/include/" ${RELY_INCL_DIR})
	set(RELY_LINK_DIR "C:/aris/aris-1.5.0/lib/debug/" ${RELY_LINK_DIR})
	set(RELY_LINK_LIB aris_robot aris_server aris_sensor aris_plan aris_dynamic aris_control aris_core ${RELY_LINK_LIB})
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
endif(WIN32)

set(ALL_INCL_DIR ${RELY_INCL_DIR})
set(ALL_LINK_DIR ${RELY_LINK_DIR})
set(ALL_LINK_LIB ${RELY_LINK_LIB})

include_directories(${ALL_INCL_DIR})
link_directories(${ALL_LINK_DIR})

set(SOURCE_FILES src/main.cpp src/rokae.h src/rokae.cpp src/iir.h src/iir.cpp)
add_executable(aris_kaanh ${SOURCE_FILES}) 
target_link_libraries(aris_kaanh ${ALL_LINK_LIB})
