cmake_minimum_required(VERSION 3.0.2)
project(bimu_rgbd_camera)

set(x86_flag TRUE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bimu_rgbd_camera
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../../../HD-DM-Linux-SDK-5.0.1.16/eSPDI

)


add_executable(bimu_rgbd_camera
	src/main.cpp 
	src/ColorPaletteGenerator.cpp  
	src/RegisterSettings.cpp 
	src/PlyWriter.cpp 
)

if(x86_flag)
  target_link_libraries(bimu_rgbd_camera
    ${PROJECT_SOURCE_DIR}/../../../HD-DM-Linux-SDK-5.0.1.16/eSPDI/libeSPDI_X86_64.so.5.0.1.16
    ${PROJECT_SOURCE_DIR}/../../../HD-DM-Linux-SDK-5.0.1.16/eSPDI/libeSPDI_X86_64.so
    
    ${catkin_LIBRARIES}
    -lrt -lm -ldl -lpthread -ludev
    #-ldl #-lrt #-lpthread #-lx11 #-ludev
  )
else()
  target_link_libraries(bimu_rgbd_camera
    ${PROJECT_SOURCE_DIR}/../../../HD-DM-Linux-SDK-5.0.1.16/eSPDI/libeSPDI_NVIDIA_64.so.5.0.1.12
    ${PROJECT_SOURCE_DIR}/../../../HD-DM-Linux-SDK-5.0.1.16/eSPDI/libeSPDI_NVIDIA_64.so
    
    ${catkin_LIBRARIES}
    -lrt -lm -ldl -lpthread -ludev
    #-ldl #-lrt #-lpthread #-lx11 #-ludev
  )

endif()


## Declare a catkin package
catkin_package()



