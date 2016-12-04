cmake_minimum_required(VERSION 2.8.3)
project(ndt_feature)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  rosconsole 
  ndt_registration 
  ndt_map 
  ndt_mcl
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
  das
  message_generation
  geometry_msgs
)


add_message_files(
  FILES
  NDTGraphMsg.msg
  NDTEdgeMsg.msg   
  NDTNodeMsg.msg
  NDTFeatureFuserHMTMsg.msg
  
)

generate_messages(
  DEPENDENCIES 
  geometry_msgs std_msgs ndt_map
)



add_executable(ndt_feature2d_view src/ndt_feature2d_view.cpp)
add_executable(ndt_feature2d_fuser src/ndt_feature2d_fuser.cpp)
add_executable(ndt_feature_debug src/ndt_feature_debug.cpp)
add_executable(ndt_odom_debug src/ndt_odom_debug.cpp)
# add_executable(ndt_feature_pcl_eval src/ndt_feature_pcl_eval.cpp)
#add_executable(narf_keypoint_extractor src/narf_keypoint_extractor.cpp)
# add_executable(correspondence_grouping src/correspondence_grouping.cpp)
add_executable(shot_test src/shot_test.cpp)

add_executable(laser2d_extrinsic_calibration src/laser2d_extrinsic_calibration.cpp)
add_executable(ndt_feature_graph_opt src/ndt_feature_graph_opt.cpp)
add_executable(ndt_feature_map_test src/ndt_feature_map_test.cpp)
add_executable(ndt_feature_node_test src/ndt_feature_node_test.cpp)

add_executable(utils_affine_test src/utils_affine_test.cpp)
add_executable(ndt_fusion_test src/ndt_fusion_test.cpp)
add_executable(ndt_memcheck_test src/ndt_memcheck_test.cpp)
add_executable(ndt_feature_mcl_node src/ndt_feature_mcl_node.cpp)


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
include_directories(malcolm_addons/include)

find_package(OpenMP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

FIND_PACKAGE(VTK REQUIRED NO_MODULE)
IF (VTK_FOUND)
  INCLUDE (${VTK_USE_FILE})
ELSE(VTK_FOUND)
  MESSAGE(FATAL_ERROR "Cannot build the executable without VTK. Please set the VTK_DIR")
ENDIF (VTK_FOUND)

include_directories(${Boost_INCLUDE_DIRS})

include_directories(/usr/local/include/flirtlib)

target_link_libraries(ndt_feature2d_view ${Boost_LIBRARIES} ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry 
#-lgui 
-lsensors -lsensorstream -lutils_flirt -lutils)

target_link_libraries(ndt_feature2d_fuser ${Boost_LIBRARIES} ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry 
#-lgui 
-lsensors -lsensorstream -lutils_flirt -lutils /usr/local/lib/libisam.a -lcholmod)

target_link_libraries(ndt_feature_debug ${Boost_LIBRARIES} ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry 
#-lgui 
-lsensors -lsensorstream -lutils_flirt -lutils)
target_link_libraries(ndt_odom_debug ${Boost_LIBRARIES} ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry 
#-lgui 
-lsensors -lsensorstream -lutils_flirt -lutils)


# target_link_libraries(ndt_feature_pcl_eval ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${vtk_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt -lutils)

#target_link_libraries(narf_keypoint_extractor ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${MRPT_LIBS} ${Boost_LIBRARIES} ${catkin_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so)

# target_link_libraries(correspondence_grouping ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${vtk_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt -lutils)
# vtkCommon ${vtkFiltersCore_LIBRARIES} vtkRendering)
target_link_libraries(shot_test ${Boost_LIBRARIES} ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry 
#-lgui 
-lsensors -lsensorstream -lutils_flirt -lutils)


target_link_libraries(laser2d_extrinsic_calibration ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${vtk_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry 
#-lgui 
-lsensors -lsensorstream -lutils_flirt -lutils)

target_link_libraries(ndt_feature_graph_opt ${Boost_LIBRARIES} ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry #-lgui 
-lsensors -lsensorstream -lutils_flirt -lutils /usr/local/lib/libisam.a -lcholmod)

target_link_libraries(ndt_feature_map_test ${Boost_LIBRARIES} ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry 
#-lgui 
-lsensors -lsensorstream -lutils_flirt)

target_link_libraries(ndt_feature_node_test ${Boost_LIBRARIES} ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry 
#-lgui 
-lsensors -lsensorstream -lutils_flirt -lutils)

target_link_libraries(utils_affine_test  ${Boost_LIBRARIES} ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry 
#-lgui 
-lsensors -lsensorstream -lutils_flirt -lutils)
target_link_libraries(ndt_fusion_test ${Boost_LIBRARIES} ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry #-lgui 
-lsensors -lsensorstream -lutils_flirt -lutils)

target_link_libraries(ndt_memcheck_test ${Boost_LIBRARIES} ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry 
#-lgui 
-lsensors -lsensorstream -lutils_flirt)

target_link_libraries(ndt_feature_mcl_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry #-lgui 
-lsensors -lsensorstream -lutils_flirt -lutils)


## Better graph 

set(CMAKE_CXX_STANDARD 11)

add_subdirectory(cmake_modules)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules/")

find_package(bettergraph)
# find_package(G2O REQUIRED)
# find_package(SUITESPARSE REQUIRED)
find_package(vodigrex)
# find_package(Cpd REQUIRED)
include_directories(${vodigrex_INCLUDE_DIRS})
# INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})
# INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR})

# set(G2O_LIBS ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE}  ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CHOLMOD_LIBRARY}  ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_TYPES_SLAM2D})

set(NDT_FEATURE_LIBS -L/usr/local/lib/flirtlib -lfeature -lgeometry 
#-lgui 
-lsensors -lsensorstream -lutils_flirt -lutils)

# ADD_LIBRARY(tutorial_slam2d_library_real malcolm_addons/Test/simulator.h malcolm_addons/Test/simulator.cpp ${G2O_LIBS})

catkin_package(
    DEPENDS eigen libpcl-all-dev mrpt glut libxmu-dev 
    CATKIN_DEPENDS ndt_registration ndt_map ndt_mcl laser_geometry sensor_msgs ndt_visualisation std_srvs flirtlib_ros das message_runtime semrob_generic semrob_conversions semrob_rviz semrob_geometry eigen_conversions geometry_msgs
    INCLUDE_DIRS include /usr/local/include/flirtlib ${vodigrex_INCLUDE_DIRS}
    LIBRARIES ${vodigrex_LIBRARIES} # TODO
)


# if(bettergraph_FOUND)
# # Libs were not linked
# 	add_subdirectory(malcolm_addons)

# set(CMAKE_CXX_STANDARD 11)

# find_package(vodigrex)
# include_directories(${vodigrex_INCLUDE_DIRS})


# find_package(Cpd REQUIRED)

# add_subdirectory(cmake_modules)

# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#  message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()


#Add CMake module path
message("Moduele path")
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules/")
message(STATUS "${CMAKE_MODULE_PATH}=${${CMAKE_MODULE_PATH}}")

# find_package(G2O REQUIRED)
# # find_package(CSPARSE REQUIRED)
# # find_package(CHOLMOD REQUIRED)
# find_package(SUITESPARSE REQUIRED)

# if(CHOLMOD_FOUND)
# 	message("FOUND CHOL YEAH")
# else(CHOLMOD_FOUND)
# 	message("NOT FOUND CHOL YEAH")
# endif(CHOLMOD_FOUND)

# INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})
# INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR})
message("Added includes")
# INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

# list(APPEND CMAKE_PREFIX_PATH "/home/malcolm/external_source/SuiteSparse/lib")

# message(${G2O_SOLVER_CSPARSE_EXTENSION})

# ADD_LIBRARY(tutorial_slam3d ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE} ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CHOLMOD_LIBRARY}  ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_TYPES_SLAM3D})


message("Adding executable")
# add_executable(g2o_test malcolm_addons/Test/test_g2o.cpp)
# target_link_libraries(g2o_test tutorial_slam2d_library_real ${G2O_LIBS})

# add_executable(manual_graph malcolm_addons/Test/manual_graph.cpp)
# target_link_libraries(manual_graph tutorial_slam2d_library_real ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} ${G2O_LIBS})
# add_dependencies(manual_graph ndt_feature_generate_messages_cpp)

# add_executable(ndt_mapping malcolm_addons/Test/ndt_feature_g20.cpp)
# target_link_libraries(ndt_mapping ${catkin_LIBRARIES} ${bettergraph_LIBRARIES} ${NDT_FEATURE_LIBS} /usr/local/lib/libisam.a -lcholmod tutorial_slam2d_library_real ${G2O_LIBS} 
# )

# add_executable(ndt_g2omaker malcolm_addons/src/ndt_g2omaker.cpp)
# target_link_libraries(ndt_g2omaker ${catkin_LIBRARIES} ${bettergraph_LIBRARIES} ${NDT_FEATURE_LIBS} /usr/local/lib/libisam.a -lcholmod tutorial_slam2d_library_real ${G2O_LIBS} 
# )

add_executable(test_auto_complete malcolm_addons/Test/test_auto_complete.cpp)
target_link_libraries(test_auto_complete 
${catkin_LIBRARIES} ${bettergraph_LIBRARIES} ${NDT_FEATURE_LIBS} /usr/local/lib/libisam.a -lcholmod 
# tutorial_slam2d_library_real 
# ${G2O_LIBS}
${OpenCV_LIBS} 
# ${PCL_LIBRARIES} 
${vodigrex_LIBRARIES} 
# Cpd::Library-C++ #<- NOT COMPATIBLE WITH THE REST I DON'T KNOW WHY !
)
add_dependencies(test_auto_complete ndt_feature_generate_messages_cpp)

# add_executable(ndt_g2olandmark malcolm_addons/Test/ndt_g2olandmark.cpp)
# target_link_libraries(ndt_g2olandmark ${catkin_LIBRARIES} ${bettergraph_LIBRARIES} ${NDT_FEATURE_LIBS} /usr/local/lib/libisam.a -lcholmod tutorial_slam2d_library_real ${G2O_LIBS} 
# )

# add_executable(ndt_mapping_3d malcolm_addons/Test/ndt_graph3dg2o.cpp)
# target_link_libraries(ndt_mapping_3d ${catkin_LIBRARIES} ${bettergraph_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt -lutils /usr/local/lib/libisam.a -lcholmod ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE}  ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CHOLMOD_LIBRARY}  ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_TYPES_SLAM3D} 
# )

# add_executable(airport_ndt_g2o malcolm_addons/Test/airport_g2o.cpp)
# target_link_libraries(airport_ndt_g2o ${catkin_LIBRARIES} ${bettergraph_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt -lutils /usr/local/lib/libisam.a -lcholmod tutorial_slam2d_library_real ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE}  ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CHOLMOD_LIBRARY}  ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_TYPES_SLAM2D} 
# )

# add_executable(test_conversion malcolm_addons/Test/test_conversion.cpp)
# target_link_libraries(test_conversion ${catkin_LIBRARIES} ${bettergraph_LIBRARIES} ${NDT_FEATURE_LIBS} /usr/local/lib/libisam.a -lcholmod tutorial_slam2d_library_real ${G2O_LIBS}
# ${OpenCV_LIBS} 
# ${PCL_LIBRARIES} 
# ${vodigrex_LIBRARIES} 
# Cpd::Library-C++ #<- NOT COMPATIBLE WITH THE REST I DON'T KNOW WHY !
# )

# add_executable(test_cov malcolm_addons/Test/test_cov.cpp)
# target_link_libraries(test_cov ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt -lutils)


# add_executable(test_prior_g2o malcolm_addons/Test/test_prior_g2o.cpp)
# target_link_libraries(test_prior_g2o ${catkin_LIBRARIES} ${bettergraph_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt -lutils /usr/local/lib/libisam.a -lcholmod tutorial_slam2d_library_real ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE}  ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CHOLMOD_LIBRARY}  ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_TYPES_SLAM2D} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${vodigrex_LIBRARIES} 
# Cpd::Library-C++
# )


# add_executable(all_real malcolm_addons/src/all_real.cpp)
# target_link_libraries(all_real ${catkin_LIBRARIES} ${bettergraph_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt -lutils /usr/local/lib/libisam.a -lcholmod tutorial_slam2d_library_real ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE}  ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CHOLMOD_LIBRARY}  ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_TYPES_SLAM2D} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${vodigrex_LIBRARIES} 
# Cpd::Library-C++
# )


# add_executable(test_read_g2p malcolm_addons/Test/test_read_g2p.cpp)
# target_link_libraries(test_read_g2p ${catkin_LIBRARIES} ${bettergraph_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt -lutils /usr/local/lib/libisam.a -lcholmod tutorial_slam2d_library_real ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE}  ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CHOLMOD_LIBRARY}  ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_TYPES_SLAM2D} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${vodigrex_LIBRARIES} 
# Cpd::Library-C++
# )

# add_executable(test_covariance_graph_g2o malcolm_addons/Test/test_covariance_graph_g2o.cpp)
# target_link_libraries(test_covariance_graph_g2o 
# ${catkin_LIBRARIES} ${bettergraph_LIBRARIES} ${NDT_FEATURE_LIBS} /usr/local/lib/libisam.a -lcholmod tutorial_slam2d_library_real ${G2O_LIBS}
# ${OpenCV_LIBS} 
# ${PCL_LIBRARIES} 
# ${vodigrex_LIBRARIES} 
# Cpd::Library-C++ #<- NOT COMPATIBLE WITH THE REST I DON'T KNOW WHY !
# )

# add_executable(test_same_graph_bigger malcolm_addons/Test/test_same_graph_bigger.cpp)
# target_link_libraries(test_same_graph_bigger 
# ${catkin_LIBRARIES} ${bettergraph_LIBRARIES} ${NDT_FEATURE_LIBS} /usr/local/lib/libisam.a -lcholmod tutorial_slam2d_library_real ${G2O_LIBS}
# ${OpenCV_LIBS} 
# ${PCL_LIBRARIES} 
# ${vodigrex_LIBRARIES} 
# Cpd::Library-C++ #<- NOT COMPATIBLE WITH THE REST I DON'T KNOW WHY !
# )

# endif(bettergraph_FOUND)

# add_executable(ndt_g2olandmark malcolm_addons/Test/ndt_g2olandmark.cpp)
# target_link_libraries(ndt_g2olandmark ${catkin_LIBRARIES} ${bettergraph_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt -lutils /usr/local/lib/libisam.a -lcholmod tutorial_slam2d_library_real ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE}  ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CHOLMOD_LIBRARY}  ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_TYPES_SLAM2D} 
# )

# add_subdirectory(malcolm_addons/src)
