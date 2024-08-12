cmake_minimum_required(VERSION 3.5)

project(ydlidar_os30a VERSION 0.0.0.4)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

message("ROS2 version : ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}.${PROJECT_VERSION_TWEAK}")

configure_file (
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/versionInfo.h.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/versionInfo.h"
)

set(DEPENDENCIES
    rclcpp
    rclcpp_components
    image_transport
    std_msgs
    tf2
    tf2_ros
    sensor_msgs
    OpenCV
    cv_bridge
)
# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    message("OpenCV is found.")
    set(WITH_OPENCV TRUE)
    add_definitions(-DWITH_OPENCV)
    message("Found OpenCV: ${OpenCV_VERSION}")
    message(STATUS "Defined WITH_OPENCV.")
endif()

###############################################################################
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})

###############################################################################
# INCLUDES and LIBS
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# set eys3d wrapper header files
set(eys3d_depth_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/eYs3D_wrapper/lib/eSPDI/"
    "${CMAKE_CURRENT_SOURCE_DIR}/eYs3D_wrapper/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/eYs3D_wrapper/include/DMPreview_utility"
    "${CMAKE_CURRENT_SOURCE_DIR}/eYs3D_wrapper/include/DMPreview_utility/hidapi"
    "${CMAKE_CURRENT_SOURCE_DIR}/eYs3D_wrapper/include/devices/model"
    "${CMAKE_CURRENT_SOURCE_DIR}/eYs3D_wrapper/include/video")

message(STATUS "eys3d_depth_INCLUDE_DIRS= ${eys3d_depth_INCLUDE_DIRS}")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${eys3d_depth_INCLUDE_DIRS}
)

# Submodule eys3d_wrapper_prebuilt_linux
if(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    set(ESPDI_PLATFORM_NAME X86_64)
elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    set(ESPDI_PLATFORM_NAME NVIDIA_64)
endif()

# set eys3d wrapper library
set(eys3d_depth_LIB "${CMAKE_CURRENT_SOURCE_DIR}/eYs3D_wrapper/lib/libeys3d.${ESPDI_PLATFORM_NAME}.so")
message(STATUS "eys3d_depth_library= ${eys3d_depth_LIB}")

link_directories(${eys3d_depth_LIB})

set(LINK_LIBRARIES
    ${OpenCV_LIBS}
    ${eys3d_depth_LIB}
)

###############################################################################
# set apc camera header and source files path
set(APC_CAMERA_INC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/apc_camera_component.hpp
)

set(APC_CAMERA_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/apc_camera_component.cpp
)

###############################################################################
# Bin and Install for apc_camera_component

add_library(apc_camera_component SHARED
    ${APC_CAMERA_INC}
    ${APC_CAMERA_SRC}
)
target_compile_definitions(apc_camera_component
    PRIVATE "COMPOSITION_BUILDING_DLL"
)
target_link_libraries(apc_camera_component
  ${LINK_LIBRARIES}
)
ament_target_dependencies(apc_camera_component
    ${DEPENDENCIES}
)

rclcpp_components_register_nodes(apc_camera_component "dmpreview::ApcCamera")
set(node_plugins "${node_plugins}dmpreview::ApcCamera;$<TARGET_FILE:apc_camera_component>\n")

###############################################################################
# Bin and Install for apc_camera_node
set(APC_WRAPPER_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/apc_wrapper.cpp
)

add_executable(apc_camera_node
    ${APC_WRAPPER_SRC}
)

target_link_libraries(apc_camera_node
  apc_camera_component
)

ament_target_dependencies(apc_camera_node  
  rclcpp
  rclcpp_components
)

###############################################################################
# Bin and Install for apc_dm_preview_listener
add_executable(apc_camera_listener_node src/apc_camera_listener.cpp)

target_link_libraries(apc_camera_listener_node
    apc_camera_component
)

ament_target_dependencies(apc_camera_listener_node rclcpp image_transport)

###############################################################################
# Install apc_camera_listener_node node
install(TARGETS
  apc_camera_listener_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install apc_camera_component
install(TARGETS
  apc_camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

# Install apc_camera_node node
install(TARGETS
  apc_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install eys3d wrapper libs
install(DIRECTORY 
    ${CMAKE_CURRENT_SOURCE_DIR}/eYs3D_wrapper/lib/
    DESTINATION lib/
)

# Install header files
install(DIRECTORY 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/eYs3D_wrapper/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/eYs3D_wrapper/include/DMPreview_utility/
    DESTINATION include/${PROJECT_NAME}/
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz files
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install eys3d cfg files
install(
  DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/eYs3D_wrapper/eYs3D
  DESTINATION ../../
)

install(
  DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

###############################################################################
ament_export_include_directories(include include/${PROJECT_NAME} include/${PROJECT_NAME}/DMPreview_utility/hidapi )
ament_export_libraries(apc_camera_component)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
