# add_executable(test_conversion_bettergraph test_conversion.cpp)
# target_link_libraries(test_conversion_bettergraph ${catkin_LIBRARIES} ${bettergraph_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt -lutils)

add_executable(test_cov test_cov.cpp)
target_link_libraries(test_cov ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt -lutils)