SET(feature_SRCS 
    InterestPoint.cpp 
    MultiScaleDetector.cpp
    RangeDetector.cpp
    NormalDetector.cpp
    NormalEdgeDetector.cpp
    NormalBlobDetector.cpp
    CurvatureDetector.cpp
    ShapeContext.cpp
    BetaGrid.cpp
    AbstractFeatureSetMatcher.cpp
    RansacFeatureSetMatcher.cpp
    RansacMultiFeatureSetMatcher.cpp
) 

SET(feature_HDRS
    Detector.h
    Descriptor.h
    InterestPoint.h
    MultiScaleDetector.h
    RangeDetector.h
    NormalDetector.h
    NormalEdgeDetector.h
    NormalBlobDetector.h
    CurvatureDetector.h
    ShapeContext.h
    BetaGrid.h
    AbstractFeatureSetMatcher.h
    RansacFeatureSetMatcher.h
    RansacMultiFeatureSetMatcher.h
) 

ADD_LIBRARY(feature_flirt STATIC ${feature_SRCS})
ADD_LIBRARY(feature SHARED ${feature_SRCS})

TARGET_LINK_LIBRARIES(feature boost_serialization)

install(TARGETS feature
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/flirtlib
    ARCHIVE DESTINATION lib/flirtlib)

install(TARGETS feature_flirt
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/flirtlib
    ARCHIVE DESTINATION lib/flirtlib)

install(FILES ${feature_HDRS} DESTINATION include/flirtlib/feature)
