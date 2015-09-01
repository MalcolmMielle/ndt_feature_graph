The Boost version of the graph library that is the default installation that Indigo utilize has a severe bug. In order to compile the flirtlibrary the 'dijsktra_shotest_paths.hpp has to be replaced with the one in this directory.

In order to compile the flirtlib (not a ROS package) simply
cd flirtlib
mkdir build
cd build
cmake ..
make
sudo make install

The reason for having the flirt lib package here is that some modification were needed inorder to compile it. The flirtlib_ros package was originally built for an earlier version and is here simply extended to work with Indigo.
