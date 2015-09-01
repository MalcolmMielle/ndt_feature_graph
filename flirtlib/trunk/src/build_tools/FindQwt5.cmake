# Find the Qwt includes and library, either the version linked to Qt3 or the version linked to Qt4
#
# On Windows it makes these assumptions:
#    - the Qwt DLL is where the other DLLs for Qt are (QT_DIR\bin) or in the path
#    - the Qwt .h files are in QT_DIR\include\Qwt or in the path
#    - the Qwt .lib is where the other LIBs for Qt are (QT_DIR\lib) or in the path
#
# Qwt_INCLUDE_DIR - Where to find qwt.h if Qwt
# Qwt_VERSION     - The version of the library installed
# Qwt-Qt4_LIBRARY - The Qwt library linked against Qt4 (if it exists)
# Qwt-Qt3_LIBRARY - The Qwt library linked against Qt3 (if it exists)
# Qwt-Qt4_FOUND   - Qwt was found and uses Qt4
# Qwt-Qt3_FOUND   - Qwt was found and uses Qt3
# Qwt_FOUND - Set to TRUE if Qwt was found (linked either to Qt3 or Qt4)

# Copyright (c) 2012, Gian Diego Tipaldi, <tipaldi@informatik.uni-freiburg.de>
# Copyright (c) 2007, Pau Garcia i Quiles, <pgquiles@elpauer.org>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

# Condition is "(A OR B) AND C", CMake does not support parentheses but it evaluates left to right
IF(Qwt-Qt4_LIBRARY OR Qwt-Qt3_LIBRARY AND Qwt_INCLUDE_DIR)
    SET(Qwt_FIND_QUIETLY TRUE)
ENDIF(Qwt-Qt4_LIBRARY OR Qwt-Qt3_LIBRARY AND Qwt_INCLUDE_DIR)

FIND_PACKAGE( Qt4 REQUIRED QUIET )

# Is Qwt installed? Look for header files
FIND_PATH( Qwt_INCLUDE_DIR qwt.h PATHS ${QT_INCLUDE_DIR} PATH_SUFFIXES qwt qwt5 qwt-qt4 qwt5-qt4 qwt-qt3 qwt5-qt3 include qwt/include qwt5/include qwt-qt4/include qwt5-qt4/include qwt-qt3/include qwt5-qt3/include ENV PATH)
	
	# Find Qwt version
IF( Qwt_INCLUDE_DIR )
	IF( QT4_FOUND )
		FILE( READ ${Qwt_INCLUDE_DIR}/qwt_global.h QWT_GLOBAL_H )
		STRING(REGEX REPLACE ".*#define *QWT_VERSION *(0x0[0-9]*).*" "\\1" Qwt_VERSION ${QWT_GLOBAL_H})
		
		# Find Qwt library linked to Qt4
		FIND_LIBRARY( Qwt-Qt4_TENTATIVE_LIBRARY NAMES qwt5-qt4 qwt-qt4 qwt5 qwt )
		IF( UNIX AND NOT CYGWIN)
			IF( Qwt-Qt4_TENTATIVE_LIBRARY )
				#MESSAGE("Qwt-Qt4_TENTATIVE_LIBRARY = ${Qwt-Qt4_TENTATIVE_LIBRARY}")
				EXECUTE_PROCESS( COMMAND "ldd" ${Qwt-Qt4_TENTATIVE_LIBRARY} OUTPUT_VARIABLE Qwt-Qt4_LIBRARIES_LINKED_TO )
				STRING( REGEX MATCH "QtCore" Qwt_IS_LINKED_TO_Qt4 ${Qwt-Qt4_LIBRARIES_LINKED_TO})
				IF( Qwt_IS_LINKED_TO_Qt4 )
					SET( Qwt-Qt4_LIBRARY ${Qwt-Qt4_TENTATIVE_LIBRARY} )
					SET( Qwt-Qt4_FOUND TRUE )
					IF (NOT Qwt_FIND_QUIETLY)
						MESSAGE( STATUS "Found Qwt version ${Qwt_VERSION} linked to Qt4" )
					ENDIF (NOT Qwt_FIND_QUIETLY)
				ENDIF( Qwt_IS_LINKED_TO_Qt4 )
			ENDIF( Qwt-Qt4_TENTATIVE_LIBRARY )
		ELSE( UNIX AND NOT CYGWIN)
		# Assumes qwt.dll is in the Qt dir
			SET( Qwt-Qt4_LIBRARY ${Qwt-Qt4_TENTATIVE_LIBRARY} )
			SET( Qwt-Qt4_FOUND TRUE )
			IF (NOT Qwt_FIND_QUIETLY)
				MESSAGE( STATUS "Found Qwt version ${Qwt_VERSION} linked to Qt4" )
			ENDIF (NOT Qwt_FIND_QUIETLY)
		ENDIF( UNIX AND NOT CYGWIN)
		
	ELSE( QT4_FOUND )		
		# Find Qwt library linked to Qt3
		FIND_LIBRARY( Qwt-Qt3_TENTATIVE_LIBRARY NAMES qwt-qt3 qwt qwt5-qt3 qwt5 )
		IF( UNIX AND NOT CYGWIN)
			IF( Qwt-Qt3_TENTATIVE_LIBRARY )
				#MESSAGE("Qwt-Qt3_TENTATIVE_LIBRARY = ${Qwt-Qt3_TENTATIVE_LIBRARY}")
				EXECUTE_PROCESS( COMMAND "ldd" ${Qwt-Qt3_TENTATIVE_LIBRARY} OUTPUT_VARIABLE Qwt-Qt3_LIBRARIES_LINKED_TO )
				STRING( REGEX MATCH "qt-mt" Qwt_IS_LINKED_TO_Qt3 ${Qwt-Qt3_LIBRARIES_LINKED_TO})
				IF( Qwt_IS_LINKED_TO_Qt3 )
					SET( Qwt-Qt3_LIBRARY ${Qwt-Qt3_TENTATIVE_LIBRARY} )
					SET( Qwt-Qt3_FOUND TRUE )
					IF (NOT Qwt_FIND_QUIETLY)
						MESSAGE( STATUS "Found Qwt version ${Qwt_VERSION} linked to Qt3" )
					ENDIF (NOT Qwt_FIND_QUIETLY)
				ENDIF( Qwt_IS_LINKED_TO_Qt3 )
			ENDIF( Qwt-Qt3_TENTATIVE_LIBRARY )
		ELSE( UNIX AND NOT CYGWIN)
			SET( Qwt-Qt3_LIBRARY ${Qwt-Qt3_TENTATIVE_LIBRARY} )
			SET( Qwt-Qt3_FOUND TRUE )
			IF (NOT Qwt_FIND_QUIETLY)
				MESSAGE( STATUS "Found Qwt version ${Qwt_VERSION} linked to Qt3" )
			ENDIF (NOT Qwt_FIND_QUIETLY)
		ENDIF( UNIX AND NOT CYGWIN)
		
		IF( Qwt-Qt4_FOUND OR Qwt-Qt3_FOUND )
			SET( Qwt_FOUND TRUE )
		ENDIF( Qwt-Qt4_FOUND OR Qwt-Qt3_FOUND )
		
		MARK_AS_ADVANCED( Qwt_INCLUDE_DIR Qwt-Qt4_LIBRARY Qwt-Qt3_LIBRARY )

	ENDIF( QT4_FOUND )
   	IF (NOT Qwt_FOUND AND Qwt_FIND_REQUIRED)
      		MESSAGE(FATAL_ERROR "Could not find Qwt")
   	ENDIF (NOT Qwt_FOUND AND Qwt_FIND_REQUIRED)
ENDIF( Qwt_INCLUDE_DIR )
