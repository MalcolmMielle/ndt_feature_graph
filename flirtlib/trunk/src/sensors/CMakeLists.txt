SET(sensors_SRCS 
    AbstractReading.cpp 
    LaserReading.cpp
) 

SET(sensors_HDRS 
    AbstractReading.h
    LaserReading.h
) 

ADD_LIBRARY(sensors_flirt STATIC ${sensors_SRCS})
ADD_LIBRARY(sensors SHARED ${sensors_SRCS})

TARGET_LINK_LIBRARIES(sensors geometry)

install(TARGETS sensors 
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/flirtlib
    ARCHIVE DESTINATION lib/flirtlib)

install(TARGETS sensors_flirt 
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/flirtlib
    ARCHIVE DESTINATION lib/flirtlib)

install(FILES ${sensors_HDRS} DESTINATION include/flirtlib/sensors)
