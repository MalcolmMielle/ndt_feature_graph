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

#include <sensor_msgs/LaserScan.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/file.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

namespace flirtlib_ros
{

namespace sm=sensor_msgs;
namespace gu=occupancy_grid_utils;
namespace nm=nav_msgs;
namespace gm=geometry_msgs;

using std::string;
using std::vector;

class ScanSimulator
{
public:
  ScanSimulator ();
  void publishScan (const ros::TimerEvent& e);

private:

  ros::NodeHandle nh_;
  
  const double cycle_time_;
  const string global_frame_;
  const unsigned num_lasers_;
  const sm::LaserScan scanner_params_;
  const string map_file_;
  const double resolution_;

  nm::OccupancyGrid::Ptr grid_;

  tf::TransformListener tf_;
  ros::Timer timer_;
  vector<ros::Publisher> scan_pubs_;
  ros::Publisher grid_pub_;
};

sm::LaserScan getScannerParams ()
{
  sm::LaserScan params;
  const double PI = 3.14159265;
  params.angle_min = -PI/2;
  params.angle_max = PI/2;
  params.angle_increment = PI/180;
  params.range_max = 10;

  return params;
}

template <class T>
T getPrivateParam (const string& name)
{
  ros::NodeHandle nh("~");
  T val;
  const bool found = nh.getParam(name, val);
  ROS_ASSERT_MSG (found, "Could not find parameter %s", name.c_str());
  ROS_DEBUG_STREAM_NAMED ("init", "Initialized " << name << " to " << val);
  return val;
}


ScanSimulator::ScanSimulator () :
  cycle_time_(0.1), global_frame_("/map"),
  num_lasers_(getPrivateParam<int>("num_lasers")),
  scanner_params_(getScannerParams()),
  map_file_(getPrivateParam<string>("map_file")),
  resolution_(getPrivateParam<double>("resolution")),
  grid_(gu::loadGrid(map_file_, resolution_)),
  timer_(nh_.createTimer(ros::Duration(cycle_time_),
                         &ScanSimulator::publishScan, this)),
  grid_pub_(nh_.advertise<nm::OccupancyGrid>("grid", 10, true))
{
  grid_pub_.publish(grid_);
  scan_pubs_.resize(num_lasers_);
  for (unsigned i=0; i<num_lasers_; i++)
  {
    const string topic = "scan" + boost::lexical_cast<string>(i);
    scan_pubs_[i] = nh_.advertise<sm::LaserScan>(topic, 10);
  }
}


void ScanSimulator::publishScan (const ros::TimerEvent& e)
{
  for (unsigned i=0; i<num_lasers_; i++)
  {
    try
    {
      const string laser_frame = "pose" + boost::lexical_cast<string>(i);
      tf::StampedTransform trans;
      tf_.lookupTransform(global_frame_, laser_frame, ros::Time(), trans);
      gm::Pose pose;
      tf::poseTFToMsg(trans, pose);
      sm::LaserScan::Ptr scan = gu::simulateRangeScan(*grid_, pose,
                                                      scanner_params_, true);
      scan->header.frame_id = laser_frame;
      scan->header.stamp = ros::Time::now();
      scan_pubs_[i].publish(scan);
    }
    catch (tf::TransformException& e)
    {
      ROS_INFO ("Not publishing scan due to transform exception");
    }
  }
}




} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "simulate_scans");
  flirtlib_ros::ScanSimulator node;
  ros::spin();
  return 0;
}
