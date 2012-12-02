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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#rosbuild_add_gtest(test/example test/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# Declare tests
rosbuild_add_gtest(test_sim test/test_sim.cpp)
target_link_libraries(test_sim  gtest_main) # Link test_sim against gtest_main

rosbuild_add_gtest(test_real test/test_real.cpp)
target_link_libraries(test_real  gtest_main) # Link test_real against gtest_main

rosbuild_add_gtest(test_helpers test/test_helpers.cpp)
target_link_libraries(test_helpers  gtest_main) # Link test_helpers against gtest_main

# Commented out SegmentationViewer due to awkward library requirements

#add viewSegmentation executable
#include_directories("/usr/include/vtk-5.8")

#rosbuild_add_executable(SegmentationViewer src/ViewSegmentation.cpp)
#target_link_libraries (SegmentationViewer ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so )

# Add extract_objects executable
rosbuild_add_executable(extract_objects src/extract_objects.cpp)
#target_link_libraries(extract_objects ${PCL_LIBRARIES} )

# Add extract_objects rgb executable
rosbuild_add_executable(extract_objects_rgb src/extract_objects_rgb.cpp)
#target_link_libraries(extract_objects_rgb ${PCL_LIBRARIES} )

# Add measure_objects executable
rosbuild_add_executable(measure_objects src/measure_objects.cpp)
#target_link_libraries(measure_objects ${PCL_LIBRARIES} )

rosbuild_add_executable(measure_objects_rgb src/measure_objects_rgb.cpp)
#target_link_libraries(measure_objects_rgb ${PCL_LIBRARIES} )

#Add extract_objects server
rosbuild_add_executable(extract_objects_server src/extract_objects_srv.cpp)
#target_link_libraries(extract_objects_server ${PCL_LIBRARIES} )

