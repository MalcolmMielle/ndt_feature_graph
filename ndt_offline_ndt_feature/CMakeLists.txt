cmake_minimum_required(VERSION 2.8.3)
project(ndt_offline_ndt_feature)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ndt_offline
  ndt_feature
)

## TODO : find a way to remove this by linking it where it should be in ndt_feature
include_directories(/usr/local/include/flirtlib)
set(NDT_FEATURE_LIBS -L/usr/local/lib/flirtlib -lfeature -lgeometry 
#-lgui 
-lsensors -lsensorstream -lutils_flirt -lutils)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ndt_offline ndt_feature 
  DEPENDS
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(LibXml2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
ADD_DEFINITIONS(-DLINUX_OS)
#find_package(ndt_visualisation)
find_package(OpenGL REQUIRED)
find_package(GLU REQUIRED)
find_package(GLUT REQUIRED)
find_package(JPEG REQUIRED)
include_directories(${JPEG_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(include)
find_package(OpenMP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

FIND_PACKAGE(VTK REQUIRED NO_MODULE)
IF (VTK_FOUND)
  INCLUDE (${VTK_USE_FILE})
ELSE(VTK_FOUND)
  MESSAGE(FATAL_ERROR "Cannot build the executable without VTK. Please set the VTK_DIR")
ENDIF (VTK_FOUND)

include_directories(
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

add_executable(ndt_graph_offline src/ndt_graph_offline.cpp)
target_link_libraries(ndt_graph_offline ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS})

add_executable(ndt_graph_logger_offline src/ndt_graph_logger_offline.cpp)
target_link_libraries(ndt_graph_logger_offline ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS})
