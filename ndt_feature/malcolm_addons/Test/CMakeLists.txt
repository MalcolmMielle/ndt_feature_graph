# list(APPEND CMAKE_PREFIX_PATH "/home/malcolm/external_source/SuiteSparse/lib")

message(${G2O_SOLVER_CSPARSE_EXTENSION})
ADD_LIBRARY(tutorial_slam2d_library_real simulator.h simulator.cpp ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE}  ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CHOLMOD_LIBRARY}  ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_TYPES_SLAM2D})

# ADD_LIBRARY(tutorial_slam3d ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE} ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CHOLMOD_LIBRARY}  ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_TYPES_SLAM3D})


message("Adding executable")
add_executable(g2o_test test_g2o.cpp)
target_link_libraries(g2o_test tutorial_slam2d_library_real ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE}  ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CHOLMOD_LIBRARY}  ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_TYPES_SLAM2D})

add_executable(manual_graph manual_graph.cpp)
target_link_libraries(manual_graph tutorial_slam2d_library_real ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE}  ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CHOLMOD_LIBRARY}  ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_TYPES_SLAM2D})

add_executable(ndt_mapping ndt_feature_g20.cpp)
target_link_libraries(ndt_mapping ${catkin_LIBRARIES} ${bettergraph_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt -lutils /usr/local/lib/libisam.a -lcholmod tutorial_slam2d_library_real ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE}  ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CHOLMOD_LIBRARY}  ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_TYPES_SLAM2D} 
)

add_executable(ndt_mapping_3d ndt_graph3dg2o.cpp)
target_link_libraries(ndt_mapping_3d ${catkin_LIBRARIES} ${bettergraph_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt -lutils /usr/local/lib/libisam.a -lcholmod ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE}  ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CHOLMOD_LIBRARY}  ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_TYPES_SLAM3D} 
)

add_executable(airport_ndt_g2o airport_g2o.cpp)
target_link_libraries(airport_ndt_g2o ${catkin_LIBRARIES} ${bettergraph_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt -lutils /usr/local/lib/libisam.a -lcholmod tutorial_slam2d_library_real ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE}  ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CHOLMOD_LIBRARY}  ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_TYPES_SLAM2D} 
)

# add_executable(test_conversion_bettergraph test_conversion.cpp)
# target_link_libraries(test_conversion_bettergraph ${catkin_LIBRARIES} ${bettergraph_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt -lutils)

add_executable(test_cov test_cov.cpp)
target_link_libraries(test_cov ${catkin_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt -lutils)