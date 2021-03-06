cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

find_package(Eigen REQUIRED)

include_directories(${EIGEN_INCLUDE_DIRS})

add_definitions(${EIGEN_DEFINITIONS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(create_object src/create_object.cpp
                                         pom/Object.cpp
                                         pom/View.cpp
                                         pom/Pipeline2D.cpp
                                         pom/PipelineGeom.cpp
                                         pom/commons.cpp
                                         pom/POM.cpp
                                         pom/FilesManager.cpp)
target_link_libraries(create_object /home/gmanfred/devel/sandbox/SiftGPU/bin/libsiftgpu.so)
rosbuild_add_executable(detect_object src/detect_object.cpp
                                         pom/Object.cpp
                                         pom/View.cpp
                                         pom/Pipeline2D.cpp
                                         pom/PipelineGeom.cpp
                                         pom/commons.cpp
                                         pom/POD.cpp)
target_link_libraries(detect_object /home/gmanfred/devel/sandbox/SiftGPU/bin/libsiftgpu.so)
rosbuild_add_executable(detect_service src/detect_service.cpp
                                         pom/Object.cpp
                                         pom/View.cpp
                                         pom/Pipeline2D.cpp
                                         pom/PipelineGeom.cpp
                                         pom/commons.cpp
                                         pom/POD.cpp)
target_link_libraries(detect_service /home/gmanfred/devel/sandbox/SiftGPU/bin/libsiftgpu.so)
rosbuild_add_executable(save_corners src/save_corners.cpp
                                     src/SaveCorners.cpp)
rosbuild_add_executable(experiment src/experiment.cpp
                                     xp/Listener.cpp
                                     xp/Gripper.cpp)
#target_link_libraries(example ${PROJECT_NAME})
