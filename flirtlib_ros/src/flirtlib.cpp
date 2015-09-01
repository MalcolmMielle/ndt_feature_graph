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

#include <flirtlib_ros/flirtlib.h>
#include <flirtlib_ros/conversions.h>
#include <boost/foreach.hpp>

namespace flirtlib_ros
{

using std::string;
namespace sm=sensor_msgs;

Detector* createDetector (SimpleMinMaxPeakFinder* peak_finder)
{
  const double scale = 5.0;
  const double dmst = 2.0;
  const double base_sigma = 0.2;
  const double sigma_step = 1.4;
  CurvatureDetector* det = new CurvatureDetector(peak_finder, scale, base_sigma,
                                                 sigma_step, dmst);
  det->setUseMaxRange(false);
  return det;
}

DescriptorGenerator* createDescriptor (HistogramDistance<double>* dist)
{
  const double min_rho = 0.02;
  const double max_rho = 0.5;
  const double bin_rho = 4;
  const double bin_phi = 12;
  BetaGridGenerator* gen = new BetaGridGenerator(min_rho, max_rho, bin_rho,
                                                 bin_phi);
  gen->setDistanceFunction(dist);
  return gen;
}


FlirtlibFeatures::FlirtlibFeatures (ros::NodeHandle nh)
{
  ROS_INFO("Note: currently ignoring ros params and using hardcoded params");
  peak_finder_.reset(new SimpleMinMaxPeakFinder(0.34, 0.001));
  histogram_dist_.reset(new SymmetricChi2Distance<double>());
  detector_.reset(createDetector(peak_finder_.get()));
  descriptor_.reset(createDescriptor(histogram_dist_.get()));
  ransac_.reset(new RansacFeatureSetMatcher(0.0599, 0.95, 0.4, 0.4,
                                            0.0384, false));
}

// Extract flirtlib features
InterestPointVec
FlirtlibFeatures::extractFeatures (sm::LaserScan::ConstPtr scan) const
{
  boost::shared_ptr<LaserReading> reading = fromRos(*scan);
  InterestPointVec pts;
  detector_->detect(*reading, pts);
  BOOST_FOREACH (InterestPoint* p, pts) 
    p->setDescriptor(descriptor_->describe(*p, *reading));
  return pts;
}



} // namespace
