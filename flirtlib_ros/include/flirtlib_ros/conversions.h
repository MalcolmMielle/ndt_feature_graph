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

#ifndef FLIRTLIB_ROS_CONVERSIONS_H
#define FLIRTLIB_ROS_CONVERSIONS_H

#include "flirtlib.h"
#include <flirtlib_ros/ScanMap.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>


namespace flirtlib_ros
{

geometry_msgs::Pose toRos(const OrientedPoint2D &pose);

OrientedPoint2D fromRos(const geometry_msgs::Pose &pose);

boost::shared_ptr<LaserReading> fromRos(const sensor_msgs::LaserScan& scan);

visualization_msgs::Marker interestPointMarkers (const std::vector<InterestPoint*>& pts,
                                                  const geometry_msgs::Pose& pose,
                                                  unsigned id=0);


std::vector<visualization_msgs::Marker> poseMarkers (const std::vector<geometry_msgs::Pose>& poses);

InterestPointRos toRos (const InterestPoint& pt);

InterestPoint* fromRos (const InterestPointRos& m);

struct RefScan
{
  sensor_msgs::LaserScan::ConstPtr scan;
  geometry_msgs::Pose pose;
  std::vector<boost::shared_ptr<InterestPoint> > pts;
  std::vector<InterestPoint*> raw_pts;

  RefScan (sensor_msgs::LaserScan::ConstPtr scan,
           const geometry_msgs::Pose& pose,
           std::vector<InterestPoint*>& pts);

};

RefScanRos toRos (const RefScan& r);

RefScan fromRos (const RefScanRos& r);

std::vector<RefScan> fromRos (const ScanMap& scan_map);

ScanMap toRos (const std::vector<RefScan>& scans);


} // namespace

#endif // include guard
