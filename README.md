# NDT Feature Suite

This is a package for doing Graph SLAM using NDT (normal distribution transform). There is 3 subpackage:

* `ndt_feature`: the GraphSLAM package
* `ndt_offline_ndt_feature`: the package to process rosbag files offline using `ndt_feature`
* `ndt_rviz_visualisation_ndt_feature`: the package to visualize the graphs on Rviz 

## Disclaimer

This package was created to use with [the Auto-Complete-Graph SLAM](https://github.com/MalcolmMielle/Auto-Complete-Graph) (yes, this is shameless advertising for my actual project). The NDTFeatureGraph suite is meant to diseappear and be replaced by something better in the close future. Feel free to use this package, but there sould not be any more update to this package.

If you use this package, please cite [this paper](http://ieeexplore.ieee.org/abstract/document/8088137/?reload=true)

> @INPROCEEDINGS{8088137,
> author={M. Mielle and M. Magnusson and H. Andreasson and A. J. Lilienthal},
> booktitle={2017 IEEE International Symposium on Safety, Security and Rescue Robotics (SSRR)},
> title={SLAM auto-complete: Completing a robot map using an emergency map},
> year={2017},
> volume={},
> number={},
> pages={35-40},
> keywords={SLAM (robots);graph theory;mobile robots;rescue robots;robot vision;SLAM auto-complete;aerial images;emergency map;graph-based SLAM;rescue missions;robot mapping;Buildings;Covariance matrices;Navigation;Simultaneous localization and mapping;Strain;Uncertainty},
> doi={10.1109/SSRR.2017.8088137},
> ISSN={},
> month={Oct},}

## How to use

check out README file in each package :).

## Dependencies

* [perception_oru](https://github.com/OrebroUniversity/perception_oru)
* [flirtlib](https://github.com/tipaldi/flirtlib)
* [isam](https://github.com/robotperception/isam)

## FAQ

1. I have problem building the gui of flirtlib, what do I do ?

Just don't build it :). Use my fork of flirtlib for that.
