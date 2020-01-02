cmake_minimum_required( VERSION 2.8 )
project( tool )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin)

find_package( Pangolin REQUIRED)

include_directories( ${Pangolin_INCLUDE_DIRS} )

# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen 
include_directories( "/usr/include/eigen3/" )

# pcl 
find_package( PCL REQUIRED COMPONENT common io filters)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )


list(REMOVE_ITEM PCL_LIBRARIES "vtkproj")

add_executable( joinMap src/joinMap.cpp)
target_link_libraries( joinMap ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable( d2PointCloud src/d2PointCloud.cpp)
target_link_libraries( d2PointCloud ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable(depth_image2depth_file src/TUM/depth_image2depth_file.cpp)
target_link_libraries(depth_image2depth_file ${OpenCV_LIBS})

add_executable( z2PointCloud src/z2PointCloud.cpp)
target_link_libraries( z2PointCloud ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable(depthmap2file_z src/depthmap2file_z.cpp)
target_link_libraries(depthmap2file_z ${OpenCV_LIBS} ${Pangolin_LIBRARIES})

add_executable(readIclDataSet src/ICL/read_icl_dataset.cpp)
target_link_libraries(readIclDataSet ${OpenCV_LIBS})

add_executable(writeIclDataSet src/ICL/write_icl_dataset.cpp)
target_link_libraries(writeIclDataSet ${OpenCV_LIBS})

add_executable( tumformat2PointCloud src/TUM/tumformat2PointCloud.cpp)
target_link_libraries( tumformat2PointCloud ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable( depth_file2PointCloud src/TUM/depth_file2PointCloud.cpp)
target_link_libraries( depth_file2PointCloud ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable(poseNum2imageName src/poseNum2imageName.cpp)

add_executable(readColmapPose src/colmap/readPose.cpp)