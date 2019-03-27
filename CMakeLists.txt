cmake_minimum_required(VERSION 3.5)
project(VO)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
# set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

############### dependencies ######################
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 3.3.1 REQUIRED)
find_package( Sophus REQUIRED )
find_package( G2O REQUIRED )

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        "/usr/include/eigen3"
        ${Sophus_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS} 
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
        ${PROJECT_SOURCE_DIR}/src/data_loader.cpp
        ${PROJECT_SOURCE_DIR}/src/config.cpp
        ${PROJECT_SOURCE_DIR}/src/camera.cpp
        ${PROJECT_SOURCE_DIR}/src/frame.cpp
        ${PROJECT_SOURCE_DIR}/src/g2o_types.cpp
        ${PROJECT_SOURCE_DIR}/src/map.cpp
        ${PROJECT_SOURCE_DIR}/src/mappoint.cpp
        ${PROJECT_SOURCE_DIR}/src/visual_odometry.cpp
)

add_executable(main main.cpp)
target_link_libraries(main
        ${PROJECT_NAME}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        g2o_core g2o_stuff g2o_types_sba
	# ${DBoW3_LIBS}
)
