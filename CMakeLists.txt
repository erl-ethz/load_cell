cmake_minimum_required(VERSION 2.8.3)
project(load_cell)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	     roscpp
	     std_msgs
	     serial)


catkin_package(
	CATKIN_DEPENDS roscpp std_msgs serial
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(wrench_reader src/wrench_reader.cpp)
target_link_libraries(wrench_reader ${catkin_LIBRARIES})
add_dependencies(wrench_reader ${PROJECT_NAME}_gencpp )

foreach(launch)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

