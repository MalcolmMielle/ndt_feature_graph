NDTFeatureFuserHMTMsg map
geometry_msgs/Pose T
std_msgs/Float64MultiArray cov

geometry_msgs/Pose Tlocal_odom #Incremental odometry between successive local maps.

geometry_msgs/Pose Tlocal_fuse # Incremental fuse estimates between sucessive local maps.

uint32 nbUpdates
float64 time_last_update