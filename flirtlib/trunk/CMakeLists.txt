# This is the main CMake configuration file, you should always do "cmake ." from here and not from subdirs

CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)

PROJECT(FLIRT CXX C)

SET(CMAKE_VERBOSE_MAKEFILE 0)

SET(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${PROJECT_SOURCE_DIR}/src/build_tools")

add_subdirectory(src)

option(BUILD_DOC "Build the documentation " ON)

#-- Add the doc target to generate the API documentation
if(BUILD_DOC)
FIND_PACKAGE(Doxygen)
  if (NOT DOXYGEN_FOUND)
    message(WARNING "Doxygen is needed to build the documentation. Please install it correctly")
  else()
    configure_file(${PROJECT_SOURCE_DIR}/doc/Doxyfile.in ${PROJECT_BINARY_DIR}/Doxyfile @ONLY IMMEDIATE)
    add_custom_target (doc COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile SOURCES ${PROJECT_BINARY_DIR}/Doxyfile)
    message(STATUS ${PROJECT_BINARY_DIR})
    install(CODE "execute_process(COMMAND \"${CMAKE_COMMAND}\" --build ${PROJECT_BINARY_DIR} --target doc)")
    message(STATUS ${PROJECT_BINARY_DIR})
    install(DIRECTORY ${PROJECT_BINARY_DIR}/doc/html DESTINATION share/flirtlib/doc)
  endif()
endif()
# Install the licenses and the README
install(FILES ${PROJECT_SOURCE_DIR}/COPYING ${PROJECT_SOURCE_DIR}/COPYING.LESSER ${PROJECT_SOURCE_DIR}/README DESTINATION share/flirtlib)

# Install the datasets
install(DIRECTORY ${PROJECT_SOURCE_DIR}/data DESTINATION share/flirtlib PATTERN ".svn" EXCLUDE)
