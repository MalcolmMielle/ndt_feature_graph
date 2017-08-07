# NDT Feature

## How to use

Modify the file `launch/gustav_laser_tf.launch` to fit your need. The bag file name is at the bottom of the launch file.

Important parameters:

* laser_topic: the laser topic for the scans
* robot_frame: frame of the robot
* tf_odom_frame: base frame of the odom. Can be the same as the robot for example.
* world_frame: the frame that is not moving 
* sensor_frame: the frame of the camera/laser...

