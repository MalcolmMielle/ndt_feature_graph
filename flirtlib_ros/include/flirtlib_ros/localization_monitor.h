/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef FLIRTLIB_ROS_LOCALIZATION_MONITOR_H
#define FLIRTLIB_ROS_LOCALIZATION_MONITOR_H

#include <occupancy_grid_utils/geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>

namespace flirtlib_ros
{

class ScanPoseEvaluator
{
public:

  ScanPoseEvaluator (const nav_msgs::OccupancyGrid& g,
                     double threshold);
  
  // Return median distance (m) from each scan point to nearest obstacle
  float operator() (const sensor_msgs::LaserScan& scan,
                    const geometry_msgs::Pose& pose) const;
  
  // Locally adjust pose to improve quality
  geometry_msgs::Pose adjustPose (const sensor_msgs::LaserScan& scan,
                                  const geometry_msgs::Pose& init_pose,
                                  double pos_range, double dpos,
                                  double dtheta) const;
  
private:

  nav_msgs::MapMetaData geom_;
  occupancy_grid_utils::DistanceField distances_;

};


} // namespace

#endif // include guard
