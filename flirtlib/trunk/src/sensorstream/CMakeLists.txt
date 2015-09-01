SET(sensorstream_SRCS 
    CarmenLog.cpp 
    LogSensorStream.cpp
) 

SET(sensorstream_HDRS
    LogReader.h
    LogWriter.h
    SensorStream.h
    CarmenLog.h 
    LogSensorStream.h
) 

SET(sensorstream_LIBS
    sensors
)

ADD_LIBRARY(sensorstream_flirt STATIC ${sensorstream_SRCS})
ADD_LIBRARY(sensorstream SHARED ${sensorstream_SRCS})

TARGET_LINK_LIBRARIES(sensorstream ${sensorstream_LIBS})

install(TARGETS sensorstream 
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/flirtlib
    ARCHIVE DESTINATION lib/flirtlib)

install(TARGETS sensorstream_flirt 
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/flirtlib
    ARCHIVE DESTINATION lib/flirtlib)

install(FILES ${sensorstream_HDRS} DESTINATION include/flirtlib/sensorstream)
