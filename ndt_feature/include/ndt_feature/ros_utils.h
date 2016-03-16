#pragma once

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/OccupancyGrid.h>

inline void moveOccupancyMap(nav_msgs::OccupancyGrid &occ_grid, const Eigen::Affine3d &pose) {

  Eigen::Affine3d map_origin;
  tf::poseMsgToEigen(occ_grid.info.origin, map_origin);
  Eigen::Affine3d new_map_origin = pose*map_origin;
  tf::poseEigenToMsg(new_map_origin, occ_grid.info.origin);
}

