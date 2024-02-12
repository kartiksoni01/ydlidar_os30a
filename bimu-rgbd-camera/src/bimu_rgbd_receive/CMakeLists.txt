cmake_minimum_required(VERSION 3.0.2)
project(bimu_rgbd_receive)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bimu_rgbd_receive
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(bimu_rgbd_receive src/bimu_rgbd_receive.cpp)
target_link_libraries(bimu_rgbd_receive ${catkin_LIBRARIES})