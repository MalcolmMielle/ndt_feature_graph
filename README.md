# NDT Feature Suite

This is a package for doing Graph SLAM using NDT (normal distribution transform). There is 3 subpackage:

* `ndt_feature`: the GraphSLAM package
* `ndt_offline_ndt_feature`: the package to process rosbag files offline using `ndt_feature`
* `ndt_rviz_visualisation_ndt_feature`: the package to visualize the graphs on Rviz 

## How to use

check out README file in each package :).

## Dependencies

* [perception_oru](https://github.com/OrebroUniversity/perception_oru)
* [flirtlib](https://github.com/tipaldi/flirtlib)
* [isam](https://github.com/robotperception/isam)

## FAQ

1. I have problem building the gui of flirtlib, what do I do ?

Just don't build it :). In the flirtlib package, go into the cmake folder and open the file `Config.cmake.in`. Then on line 9, in the `foreach`, remove `gui`. Done.
