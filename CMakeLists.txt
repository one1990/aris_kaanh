cmake_minimum_required(VERSION 2.8)

project(ethercat_test)

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

# set include and lib folders
message("using xenomai")
add_definitions(-DUSE_XENOMAI)
set(RELY_INCL_DIR "/usr/xenomai/include" ${RELY_INCL_DIR})
set(RELY_LINK_DIR "/usr/xenomai/lib" ${RELY_LINK_DIR})
set(RELY_LINK_LIB native rtdm xenomai ${RELY_LINK_LIB})

message("using etherlab")
add_definitions(-DUSE_ETHERLAB)
set(RELY_INCL_DIR "/opt/etherlab/include" ${RELY_INCL_DIR})
set(RELY_LINK_DIR "/opt/etherlab/lib" ${RELY_LINK_DIR})
set(RELY_LINK_LIB ethercat_rtdm ${RELY_LINK_LIB})

set(ALL_INCL_DIR ${ARIS_INCL_DIR} ${RELY_INCL_DIR})
set(ALL_LINK_DIR ${ARIS_LINK_DIR} ${RELY_LINK_DIR})
set(ALL_LINK_LIB ${ARIS_LINK_LIB} ${RELY_LINK_LIB})

include_directories(${ALL_INCL_DIR})
link_directories(${ALL_LINK_DIR})

add_executable(test_xenomai main.c)
target_link_libraries(test_xenomai ${RELY_LINK_LIB})
