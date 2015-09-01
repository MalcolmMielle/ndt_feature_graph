SET(geometry_SRCS 
    point.cpp 
) 

SET(geometry_HDRS 
    point.h
) 

ADD_LIBRARY(geometry_flirt STATIC ${geometry_SRCS})
ADD_LIBRARY(geometry SHARED ${geometry_SRCS})

TARGET_LINK_LIBRARIES(geometry boost_serialization)

install(TARGETS geometry
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/flirtlib
    ARCHIVE DESTINATION lib/flirtlib)

install(TARGETS geometry_flirt
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/flirtlib
    ARCHIVE DESTINATION lib/flirtlib)

install(FILES ${geometry_HDRS} DESTINATION include/flirtlib/geometry)

