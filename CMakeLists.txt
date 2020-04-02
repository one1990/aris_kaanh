cmake_minimum_required(VERSION 3.12)

project(kaanh VERSION 1.0)

# kaanh use c++ 17 standard
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")

if(UNIX)
	set(CMAKE_INSTALL_PREFIX "/usr/kaanh/kaanh-${CMAKE_PROJECT_VERSION}")
	add_custom_target(uninstall rm -r /usr/kaanh)
endif(UNIX)
if(WIN32)
	set(CMAKE_INSTALL_PREFIX "C:/kaanh/kaanh-${CMAKE_PROJECT_VERSION}")
endif(WIN32)

## set definitions to disable some warning ##
if(UNIX)
	add_definitions(-DUNIX)
endif(UNIX)
if(WIN32)
	add_definitions(-DWIN32 -D_WINSOCK_DEPRECATED_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS)
endif(WIN32)
add_definitions(-DKAANH_INSTALL_PATH="${CMAKE_INSTALL_PREFIX}")

if(UNIX)
	set(RELY_LINK_LIB atomic pthread stdc++fs)#pthread is needed for concurrency
	set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config)
elseif(WIN32)
	set(RELY_LINK_LIB "ws2_32")#ws2_32 is needed for socket
endif()

file(GLOB HEADERS include/kaanh/*.hpp include/kaanh/*.h)
file(GLOB SOURCES src/kaanh/*.cpp src/kaanh/*.c include/kaanh/*.hpp include/kaanh/*.h ${HEADERS})
add_library(kaanh_lib STATIC ${HEADERS} ${SOURCES})
target_include_directories(kaanh_lib
	PUBLIC 
		$<INSTALL_INTERFACE:include>
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
		${RELY_INCL_DIR}
)
	
set_target_properties(kaanh_lib PROPERTIES PUBLIC_HEADER "${HEADERS}")
set(kaanh_LIBRARIES ${kaanh_LIBRARIES} optimized kaanh::kaanh_lib debug kaanh::debug::kaanh_lib)
set(KAANH_LINK_LIB ${KAANH_LINK_LIB} kaanh_lib)

install(TARGETS kaanh_lib EXPORT kaanh-lib-debug PUBLIC_HEADER DESTINATION "include/kaanh" ARCHIVE CONFIGURATIONS Debug DESTINATION "lib/debug")
install(TARGETS kaanh_lib EXPORT kaanh-lib-release PUBLIC_HEADER DESTINATION "include/kaanh" ARCHIVE CONFIGURATIONS Release DESTINATION "lib/release")

if(UNIX)
	set(KAANH_LINK_LIB -Wl,--start-group ${KAANH_LINK_LIB} -Wl,--end-group)
endif(UNIX)

## install exports ##
install(EXPORT kaanh-lib-debug NAMESPACE kaanh::debug:: FILE kaanh-lib-debug.cmake DESTINATION "share/kaanh/cmake")
install(EXPORT kaanh-lib-release NAMESPACE kaanh:: FILE kaanh-lib-release.cmake DESTINATION "share/kaanh/cmake")
install(FILES include/kaanh.h DESTINATION include)

## install config ##
include(CMakePackageConfigHelpers)
set(kaanh_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include)
set(kaanh_LIBRARIES ${kaanh_LIBRARIES} ${RELY_LINK_LIB})
configure_package_config_file(config/kaanh-config.cmake.in ${PROJECT_BINARY_DIR}/kaanh-config.cmake INSTALL_DESTINATION "share/kaanh/cmake")
install(FILES ${PROJECT_BINARY_DIR}/kaanh-config.cmake DESTINATION "share/kaanh/cmake")

link_directories(${RELY_LINK_DIR})

## install ##
#if(UNIX)
#	install(TARGETS kaanh_lib
#		LIBRARY DESTINATION lib
#		ARCHIVE DESTINATION lib)
#	install(FILES src/kaanh.h DESTINATION include/kaanh)
#endif(UNIX)

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

add_executable(kaanh src/kaanh/main.cpp)
target_link_libraries(kaanh kaanh_lib)
if(UNIX)
	target_link_libraries(kaanh ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
	target_link_libraries(kaanh ${aris_LIBRARIES})
endif(WIN32)

project(kaanh_jmc)
set(SOURCE_FILES src/kaanh_jmc/oneaxis_jmc.cpp src/kaanh_jmc/oneaxis_jmc.h)
add_executable(kaanh_jmc ${SOURCE_FILES})
if(UNIX)
	target_link_libraries(kaanh_jmc ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
	target_link_libraries(kaanh_jmc ${aris_LIBRARIES})
endif(WIN32)

project(kaanh_maxon)
set(SOURCE_FILES src/kaanh_maxon/oneaxis_maxon.cpp src/kaanh_maxon/oneaxis_maxon.h)
add_executable(kaanh_maxon ${SOURCE_FILES})
if(UNIX)
        target_link_libraries(kaanh_maxon ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
        target_link_libraries(kaanh_maxon ${aris_LIBRARIES})
endif(WIN32)

project(kaanh_taike)
set(SOURCE_FILES src/kaanh_taike/oneaxis_taike.cpp src/kaanh_taike/oneaxis_taike.h)
add_executable(kaanh_taike ${SOURCE_FILES})
if(UNIX)
        target_link_libraries(kaanh_taike ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
        target_link_libraries(kaanh_taike ${aris_LIBRARIES})
endif(WIN32)

project(wuling)
set(SOURCE_FILES src/wuling/main_wuling.cpp src/wuling/wuling.cpp src/wuling/wuling.h src/wuling/planfuns.h src/wuling/planfuns.cpp src/wuling/json.hpp src/wuling/wulingconfig.cpp src/wuling/wulingconfig.h)
add_executable(wuling ${SOURCE_FILES})
if(UNIX)
        target_link_libraries(wuling ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
        target_link_libraries(wuling ${aris_LIBRARIES})
endif(WIN32)
