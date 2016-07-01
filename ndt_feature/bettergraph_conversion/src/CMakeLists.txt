
# add_executable(ndt_g2o ndt_feature_g20.cpp)
# target_link_libraries(ndt_g2o ${catkin_LIBRARIES} ${bettergraph_LIBRARIES} -L/usr/local/lib/flirtlib -lfeature -lgeometry -lgui -lsensors -lsensorstream -lutils_flirt -lutils /usr/local/lib/libisam.a -lcholmod 
# 
# ${CSPARSE_LIBRARY}
# ${CHOLMOD_LIBRARY} 
# ${CSPARSE_LIBRARIES}
# ${CHOLMOD_LIBRARIES}
# ${G2O_CORE_LIBRARY} 
# ${G2O_STUFF_LIBRARY} 
# # ${G2O_SOLVER_CSPARSE}
# ${G2O_SOLVER_SLAM2D_LINEAR}
# # ${G2O_SOLVER_CHOLMOD}
# ${G2O_TYPES_SLAM2D} 
# )

# list(APPEND CMAKE_PREFIX_PATH "/home/malcolm/external_source/SuiteSparse/lib")

message(${G2O_SOLVER_CSPARSE_EXTENSION})
ADD_LIBRARY(tutorial_slam2d_library_real simulator.h simulator.cpp ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE}  ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CHOLMOD_LIBRARY}  ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_TYPES_SLAM2D})


message("Adding executable")
add_executable(g2o_test test_g2o.cpp)
target_link_libraries(g2o_test tutorial_slam2d_library_real ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE}  ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CHOLMOD_LIBRARY}  ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_TYPES_SLAM2D})