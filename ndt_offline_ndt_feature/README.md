# NDT Offline NDT Feature

## How ot use

The code is a bit dirty so all the parameters for the graph SLAM are hardcoded in the `.cpp` file :/.

Otherwise, an example command is (replace Xs by your argument):

```
rosrun ndt_offline ndt_offline_laser --base-name X --dir-name X --velodyne_packets_topic X --tf_sensor_link X --tf_world_frame X --tf_base_link X --save_map X --use-odometry X
```

with:

* base-name: prefix for all generated files
* dir-name: where to look for SCANs, i.e. the folder with the bags
* velodyne_packets_topic: a laser topic with sensor_msgs::LaserScan type of messages
* tf_sensor_link: tf_sensor_link : tf of where the sensor is
* tf_world_frame: tf world frame
* tf_base_link: base link of the robot
* save_map: saves the map at the end of execution
* use-odometry: use the odometry or not during the SLAM


