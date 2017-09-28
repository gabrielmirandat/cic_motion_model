cmake_minimum_required(VERSION 2.8.3)
project(cic_motion_model)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# SET(GCC_COVERAGE_LINK_FLAGS    "-lboost_iostreams")

find_package(cmake_modules REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS
    iostreams)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  p2os_msgs
  tf
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs tf p2os_msgs
)

include_directories(
  include
  /usr/include/eigen3
  gnuplot-iostream-master
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN_INCLUDE_DIR}
)

add_executable( cic_motion_model_node 
            		src/main.cpp 
            		src/Node.cpp 
            		src/MotionModel.cpp
	      )
target_link_libraries(cic_motion_model_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
