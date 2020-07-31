cmake_minimum_required(VERSION 3.0.2)
project(tars_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  serial
  vicon
  mg_msgs
  actionlib_msgs
  actionlib
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf serial vicon mg_msgs actionlib_msgs actionlib
  DEPENDS Eigen
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS}
)


add_executable(circleControl src/circleControl.cpp)
target_link_libraries(circleControl serial ${catkin_LIBRARIES})

add_executable(semiCircleControl src/semiCircleControl.cpp)
target_link_libraries(semiCircleControl serial ${catkin_LIBRARIES})

add_executable(stop src/stop.cpp)
target_link_libraries(stop serial ${catkin_LIBRARIES})



add_executable(polynomialControl src/polynomialControl.cpp src/polynomials.cpp)
target_link_libraries(polynomialControl serial ${catkin_LIBRARIES})



add_executable(testPolyControl src/test_poly_control.cpp src/helper.cpp)
target_link_libraries(testPolyControl ${catkin_LIBRARIES})

add_dependencies(
	polynomialControl
	testPolyControl
	${tars_control_EXPORTED_TARGETS}
)













