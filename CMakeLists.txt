cmake_minimum_required(VERSION 3.12)

project(aris_luoshi)

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_CXX_STANDARD 17)

# for linux
# set include and lib folders
# message("using xenomai")
# add_definitions(-DUSE_XENOMAI)
# set(RELY_INCL_DIR "/usr/xenomai/include" ${RELY_INCL_DIR})
# set(RELY_LINK_DIR "/usr/xenomai/lib" ${RELY_LINK_DIR})
# set(RELY_LINK_LIB native rtdm xenomai ${RELY_LINK_LIB})

# message("using etherlab")
# add_definitions(-DUSE_ETHERLAB)
# set(RELY_INCL_DIR "/opt/etherlab/include" ${RELY_INCL_DIR})
# set(RELY_LINK_DIR "/opt/etherlab/lib" ${RELY_LINK_DIR})
# set(RELY_LINK_LIB ethercat_rtdm ${RELY_LINK_LIB})

# set(ALL_INCL_DIR ${ARIS_INCL_DIR} ${RELY_INCL_DIR})
# set(ALL_LINK_DIR ${ARIS_LINK_DIR} ${RELY_LINK_DIR})
# set(ALL_LINK_LIB ${ARIS_LINK_LIB} ${RELY_LINK_LIB})

# for win
set(RELY_LINK_LIB "ws2_32")#ws2_32 is needed for socket
set(ARIS_INCL_DIR "C:/aris/include/")
set(ARIS_LINK_DIR "C:/aris/lib/debug/")
set(ARIS_LINK_LIB aris_robot aris_server aris_sensor aris_plan aris_dynamic aris_control aris_core)
set(ALL_INCL_DIR ${ARIS_INCL_DIR})
set(ALL_LINK_DIR ${ARIS_LINK_DIR})
set(ALL_LINK_LIB ${ARIS_LINK_LIB} ${RELY_LINK_LIB})
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")


include_directories(${ALL_INCL_DIR})
link_directories(${ALL_LINK_DIR})

set(SOURCE_FILES src/main.cpp src/rokae.h src/rokae.cpp )
add_executable(aris_luoshi ${SOURCE_FILES}) 
target_link_libraries(aris_luoshi ${ALL_LINK_LIB})