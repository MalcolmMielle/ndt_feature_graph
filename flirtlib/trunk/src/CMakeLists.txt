if(NOT CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE RelWithDebInfo)
endif(NOT CMAKE_BUILD_TYPE)

# For out-of-source build in code structered with an src directory
#STRING(REGEX REPLACE "/src$" "/lib" CMAKE_LIBRARY_OUTPUT_DIRECTORY ${FLIRT_SOURCE_DIR})
#STRING(REGEX REPLACE "/src$" "/lib" CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${FLIRT_SOURCE_DIR})
#STRING(REGEX REPLACE "/src$" "/bin" CMAKE_RUNTIME_OUTPUT_DIRECTORY ${FLIRT_SOURCE_DIR})

# Set to build shared objects as option
OPTION(BUILD_SHARED_LIBS "Build package with shared libraries." ON)

# ---------------------------- Defining building options ------------------------------
option(BUILD_GUI "Build the GUI" ON)


# ---------------------------- Checking the OS ------------------------------
IF("${CMAKE_SYSTEM}" MATCHES "Linux")
	MESSAGE(STATUS "Compiling on Linux")
	SET(LINUX 1)
	EXEC_PROGRAM("uname" ARGS -m OUTPUT_VARIABLE myArch)
	IF("${myArch}" MATCHES "x86_64")
		MESSAGE(STATUS "  64-bit architecture detected")
		SET(LINUX64 1)
# 		SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}64")
# 		SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_OUTPUT_DIRECTORY}64")
	ENDIF("${myArch}" MATCHES "x86_64")
ELSEIF(APPLE)
	SET(LINUX 0)
	MESSAGE(STATUS "Compiling on MacOSX")
ELSEIF(WIN32)
	SET(LINUX 0)
	IF(CYGWIN)
		MESSAGE(STATUS "Compiling on CygWin")
		SET(CYGWIN 1)
	ELSE(CYGWIN)
		MESSAGE(FATAL_ERROR "Compiling on Windows is not possible at the moment")
	ENDIF(CYGWIN)
ENDIF("${CMAKE_SYSTEM}" MATCHES "Linux")


# ---------------------------- Checking the Libraries ------------------------------
# Library: Boost
set(Boost_USE_MULTITHREADED ON)
FIND_PACKAGE(Boost 1.36.0 COMPONENTS program_options)
#FIND_PACKAGE(Boost COMPONENTS program_options math graph)
IF(Boost_FOUND)
	MESSAGE(STATUS "Boost found")
	IF($ENV{VERBOSE})
 		MESSAGE(STATUS "   Boost_INCLUDE_DIR = ${Boost_INCLUDE_DIR}")
 		MESSAGE(STATUS "   Boost_LINK_DIRECTORIES = ${Boost_LINK_DIRECTORIES}")
		MESSAGE(STATUS "   Boost Version = m:${Boost_MINOR_VERSION} M:${Boost_MAJOR_VERSION} v:${Boost_VERSION}")
	ENDIF($ENV{VERBOSE})
	INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
	LINK_DIRECTORIES(${Boost_LINK_DIRECTORIES})
ELSE(Boost_FOUND)
	MESSAGE(FATAL_ERROR "	Boost not found, it is REQUIRED to build the FLIRT.\n Boost can be found on http://www.boost.org")
ENDIF(Boost_FOUND)

IF(BUILD_GUI)
    # Library: Qt4
    FIND_PACKAGE(Qt4)
    IF(QT_FOUND)
	    MESSAGE(STATUS "Qt4 found")
	    SET(QT_USE_QTOPENGL 1)
	    INCLUDE(${QT_USE_FILE})
	    IF($ENV{VERBOSE})
		    MESSAGE(STATUS "   QT_LIBRARIES = ${QT_LIBRARIES}")
		    MESSAGE(STATUS "   QT_INCLUDE_DIR = ${QT_INCLUDE_DIR}")
	    ENDIF($ENV{VERBOSE})
	    INCLUDE_DIRECTORIES(${QT_INCLUDE_DIR})
    ELSE(QT_FOUND)
	    MESSAGE( STATUS "	Qt4 not found, it is REQUIRED to build the Gui.")
	    SET(BUILD_GUI 0)
    ENDIF(QT_FOUND)
ENDIF(BUILD_GUI)

# Library: Qwt
IF(BUILD_GUI)
FIND_PACKAGE(Qwt)
IF(Qwt-Qt4_FOUND)
	MESSAGE(STATUS "Qwt found")
	IF($ENV{VERBOSE})
		MESSAGE(STATUS "   Qwt-Qt4_LIBRARY = ${Qwt-Qt4_LIBRARY}")
		MESSAGE(STATUS "   Qwt_INCLUDE_DIR = ${Qwt_INCLUDE_DIR}")
		MESSAGE(STATUS "   Qwt_VERSION 	   = ${Qwt_VERSION}")
	ENDIF($ENV{VERBOSE})
	INCLUDE_DIRECTORIES(${Qwt_INCLUDE_DIR})
	SET(QWT_LIBRARIES ${Qwt-Qt4_LIBRARY})
	ADD_DEFINITIONS(-DQWT_VERSION=${Qwt_VERSION})
#	INCLUDE_DIRECTORIES(/usr/local/qwt/include)
#	INCLUDE_DIRECTORIES(${QT_INCLUDE_DIR}/Qt)
#	INCLUDE_DIRECTORIES(${QT_INCLUDE_DIR}/QtGui)
#	INCLUDE_DIRECTORIES(${QT_INCLUDE_DIR}/QtCore)
#	LINK_DIRECTORIES(${QWT_INCLUDE_DIR}/../lib)
ELSE(Qwt-Qt4_FOUND)
	MESSAGE(STATUS "	Qwt not found, it is REQUIRED to build the Gui.")
	SET(BUILD_GUI 0)
ENDIF(Qwt-Qt4_FOUND)
ENDIF(BUILD_GUI)

# Library: OpenGL
IF(BUILD_GUI)
    FIND_PACKAGE(OpenGL)
    IF(OPENGL_FOUND)
	    MESSAGE(STATUS "OpenGL found")
	    IF($ENV{VERBOSE})
		    MESSAGE(STATUS "   OPENGL_LIBRARIES = ${OPENGL_LIBRARIES}")
		    MESSAGE(STATUS "   OPENGL_INCLUDE_DIR = ${OPENGL_INCLUDE_DIR}")
	    ENDIF($ENV{VERBOSE})
	    INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR}/GL)
    ELSE(OPENGL_FOUND)
	    MESSAGE(STATUS "	OpenGL not found, it is REQUIRED to build the Gui.")
	    SET(BUILD_GUI 0)
   ENDIF(OPENGL_FOUND)
ENDIF(BUILD_GUI)

# Library: CAIRO
FIND_PACKAGE(Cairo)
IF(CAIRO_FOUND)
        MESSAGE(STATUS "Cairo found")
        IF($ENV{VERBOSE})
                MESSAGE(STATUS "   CAIRO_LIBRARIES = ${CAIRO_LIBRARIES}")
                MESSAGE(STATUS "   CAIRO_INCLUDE_DIR = ${CAIRO_INCLUDE_DIR}")
        ENDIF($ENV{VERBOSE})
        INCLUDE_DIRECTORIES(${CAIRO_INCLUDE_DIR})
ELSE(CAIRO_FOUND)
        MESSAGE(STATUS "        Cairo not found, it is OPTIONAL: to draw to PDF files.")
ENDIF(CAIRO_FOUND)

INCLUDE_DIRECTORIES(${FLIRT_SOURCE_DIR}/src)

# ---------------------------- Setting compilation flags ------------------------------
# Optimization
SET(LOCAL_OPTIMIZATION_FLAGS "-O3 -march=nocona -msse4.1 -funroll-loops -ftree-vectorize -DTBB -mfpmath=both")
# SET(LOCAL_OPTIMIZATION_FLAGS "-O3 -march=native -msse4.2 -msse4.1 -funroll-loops -ftree-vectorize -DTBB -mfpmath=both")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${LOCAL_OPTIMIZATION_FLAGS}")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=gnu++0x ${LOCAL_OPTIMIZATION_FLAGS}")

#Debug (it can clash with optimization)
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 -g3 -ggdb3 -fno-inline -Wall")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=gnu++0x -O0 -g3 -ggdb3 -fno-inline -Wall")

#Profiling
#ADD_DEFINITIONS(-pg)
#SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")
#SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")


ADD_SUBDIRECTORY(geometry)
ADD_SUBDIRECTORY(sensors)
ADD_SUBDIRECTORY(sensorstream)
ADD_SUBDIRECTORY(feature)
ADD_SUBDIRECTORY(utils)

IF(BUILD_GUI)
	ADD_SUBDIRECTORY(gui)
ENDIF(BUILD_GUI)
ADD_SUBDIRECTORY(applications)

