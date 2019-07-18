cmake_minimum_required(VERSION 2.8.3)

project(rokae2)

find_package(catkin REQUIRED  COMPONENTS roscpp rospy std_msgs genmsg tf)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
#add_executable(move_Node src/move.cpp)
#target_link_libraries(move_Node ${catkin_LIBRARIES})

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
