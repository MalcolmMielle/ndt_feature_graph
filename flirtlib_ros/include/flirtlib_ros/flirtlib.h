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

#ifndef FLIRTLIB_ROS_FLIRTLIB_H
#define FLIRTLIB_ROS_FLIRTLIB_H

#define BOOST_NO_HASH

#include <flirtlib/feature/Detector.h>
#include <flirtlib/feature/ShapeContext.h>
#include <flirtlib/feature/BetaGrid.h>
#include <flirtlib/feature/RangeDetector.h>
#include <flirtlib/feature/CurvatureDetector.h>
#include <flirtlib/feature/NormalBlobDetector.h>
#include <flirtlib/feature/NormalEdgeDetector.h>
#include <flirtlib/feature/RansacFeatureSetMatcher.h>
#include <flirtlib/feature/RansacMultiFeatureSetMatcher.h>
#include <flirtlib/sensorstream/CarmenLog.h>
#include <flirtlib/sensorstream/LogSensorStream.h>
#include <flirtlib/sensorstream/SensorStream.h>
#include <flirtlib/utils/SimpleMinMaxPeakFinder.h>
#include <flirtlib/utils/HistogramDistances.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>

namespace flirtlib_ros
{

typedef std::vector<InterestPoint*> InterestPointVec;

struct FlirtlibFeatures
{
  FlirtlibFeatures (ros::NodeHandle nh = ros::NodeHandle("~"));
  
  boost::shared_ptr<SimpleMinMaxPeakFinder> peak_finder_;
  boost::shared_ptr<HistogramDistance<double> > histogram_dist_;
  boost::shared_ptr<Detector> detector_;
  boost::shared_ptr<DescriptorGenerator> descriptor_;
  boost::shared_ptr<RansacFeatureSetMatcher> ransac_;
  
  InterestPointVec extractFeatures (sensor_msgs::LaserScan::ConstPtr scan) const;
};

} // namespace

#endif // include guard
