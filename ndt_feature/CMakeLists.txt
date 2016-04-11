cmake_minimum_required(VERSION 2.8.3)
project(ndt_feature)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  rosconsole 
  ndt_registration 
  ndt_map 
  laser_geometry 
  sensor_msgs 
  ndt_visualisation 
  std_srvs 
  flirtlib_ros 
  eigen_conversions
  semrob_generic
  semrob_conversions
  semrob_rviz
  semrob_geometry
)

catkin_package(
    DEPENDS eigen libpcl-all-dev mrpt glut libxmu-dev 
    CATKIN_DEPENDS ndt_registration ndt_map laser_geometry sensor_msgs ndt_visualisation std_srvs flirtlib_ros
    INCLUDE_DIRS include
    LIBRARIES # TODO
)

add_executable(ndt_feature2d_view src/ndt_feature2d_view.cpp)
add_executable(ndt_feature2d_fuser src/ndt_feature2d_fuser.cpp)
add_executable(ndt_feature_debug src/ndt_feature_debug.cpp)
add_executable(ndt_odom_debug src/ndt_odom_debug.cpp)
add_executable(ndt_feature_pcl_eval src/ndt_feature_pcl_eval.cpp)
#add_executable(narf_keypoint_extractor src/narf_keypoint_extractor.cpp)
add_executable(correspondence_grouping src/correspondence_grouping.cpp)
add_executable(shot_test src/shot_test.cpp)

add_executable(laser2d_extrinsic_calibration src/laser2d_extrinsic_calibration.cpp)
add_executable(ndt_feature_graph_opt src/ndt_feature_graph_opt.cpp)
add_executable(ndt_feature_map_test src/ndt_feature_map_test.cpp)
add_executable(ndt_feature_node_test src/ndt_feature_node_test.cpp)

add_executable(utils_affine_test src/utils_affine_test.cpp)
add_executable(ndt_fusion_test src/ndt_fusion_test.cpp)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(LibXml2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
ADD_DEFINITIONS(-DLINUX_OS)
#find_package(ndt_visualisation)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(include)

find_package(OpenMP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

include_directories(${Boost_INCLUDE_DIRS})

include_directories(/usr/local/include/flirtlib)

target_link_libraries(ndt_feature2d_view ${Boost_LIBRARIES} ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt)

target_link_libraries(ndt_feature2d_fuser ${Boost_LIBRARIES} ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt /usr/local/lib/libisam.a -lcholmod)

target_link_libraries(ndt_feature_debug ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(ndt_odom_debug ${Boost_LIBRARIES} ${catkin_LIBRARIES})


target_link_libraries(ndt_feature_pcl_eval ${Boost_LIBRARIES} ${catkin_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so)

#target_link_libraries(narf_keypoint_extractor ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${MRPT_LIBS} ${Boost_LIBRARIES} ${catkin_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so)

target_link_libraries(correspondence_grouping ${Boost_LIBRARIES} ${catkin_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so)
target_link_libraries(shot_test ${Boost_LIBRARIES} ${catkin_LIBRARIES})


target_link_libraries(laser2d_extrinsic_calibration ${Boost_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(ndt_feature_graph_opt ${Boost_LIBRARIES} ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt  /usr/local/lib/libisam.a -lcholmod)

target_link_libraries(ndt_feature_map_test ${Boost_LIBRARIES} ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt)

target_link_libraries(ndt_feature_node_test ${Boost_LIBRARIES} ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt)

target_link_libraries(utils_affine_test  ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(ndt_fusion_test ${Boost_LIBRARIES} ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt)