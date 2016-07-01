
set(CMAKE_CXX_STANDARD 11)

# add_subdirectory(cmake_modules)

# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#  message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()

#Add CMake module path
message("Moduele path")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules/")
message(STATUS "${CMAKE_MODULE_PATH}=${${CMAKE_MODULE_PATH}}")

find_package(G2O REQUIRED)
# find_package(CSPARSE REQUIRED)
# find_package(CHOLMOD REQUIRED)
find_package(SUITESPARSE REQUIRED)

if(CHOLMOD_FOUND)
	message("FOUND CHOL YEAH")
else(CHOLMOD_FOUND)
	message("NOT FOUND CHOL YEAH")
endif(CHOLMOD_FOUND)

INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR})
message("Added includes")
# INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

include_directories(include)

add_subdirectory(include)
add_subdirectory(test)
message("INto source")
add_subdirectory(src)
