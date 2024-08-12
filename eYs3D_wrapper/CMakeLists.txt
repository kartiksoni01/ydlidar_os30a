cmake_minimum_required(VERSION 3.10)
project(eys3d)
set(CMAKE_FIND_DEBUG_MODE TRUE)
set(CMAKE_DEBUG_TARGET_PROPERTIES TRUE)

set(CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)#Release Debug RelWithDebInfo
    message(STATUS "Build type not specify, default type is Release")
endif(NOT CMAKE_BUILD_TYPE)

add_definitions(-DUSE_OPENCL)

include_directories(
        include/
        include/DMPreview_utility
        include/DMPreview_utility/sqlite3
        include/devices/
        include/devices/controller
        include/devices/model
        include/sensors
        lib/eSPDI/
        )

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/eSPDI/
        )

set(DEPENDENCY_LIBS
                eSPDI_${ESPDI_PLATFORM_NAME}
                eys3d.${ESPDI_PLATFORM_NAME})

set(CMAKE_INSTALL_RPATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Install eys3d and eYs3D.test to out folder

# Test file source
## (1) Target is callback.test
    
set(TEST_SRC src/callback_main.cpp)

add_executable(callback.test
                    ${TEST_SRC})
                    

target_link_libraries(callback.test
            ${DEPENDENCY_LIBS})
       
### (2) Target is pipeline.test
set(TEST_SRC src/pipeline_main.cpp)
add_executable(pipeline.test
        ${TEST_SRC})

target_link_libraries(pipeline.test
            ${DEPENDENCY_LIBS})



# ### (3) Target is frameset_pipeline.test
set(TEST_SRC src/frameset_pipeline_main.cpp)

add_executable(frameset_pipeline.test
                    ${TEST_SRC})

target_link_libraries(frameset_pipeline.test
            ${DEPENDENCY_LIBS})

    
# Install eys3d and eYs3D.test to out folder

install(TARGETS callback.test pipeline.test frameset_pipeline.test
            LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/out
            RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/out)

install(DIRECTORY eYs3D/ DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/out/eYs3D)


