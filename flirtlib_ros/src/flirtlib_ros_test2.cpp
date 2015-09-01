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
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

namespace sm=sensor_msgs;
namespace vm=visualization_msgs;
namespace gm=geometry_msgs;

using std::string;
using std::vector;

typedef boost::mutex::scoped_lock Lock;
typedef vector<InterestPoint*> InterestPointVec;
typedef std::pair<InterestPoint*, InterestPoint*> Correspondence;
typedef vector<Correspondence> Correspondences;

namespace flirtlib_ros
{

/************************************************************
 * Utilities
 ***********************************************************/

gm::Point toPoint (const tf::Vector3& p)
{
  gm::Point pt;
  pt.x = p.x();
  pt.y = p.y();
  pt.z = p.z();
  return pt;
}

double averageDistance (const Correspondences& corresp,
                              const gm::Pose& p0,
                              const gm::Pose& p1)
{
  const unsigned n = corresp.size();
  if (n<2)
    return 1e17;

  tf::Transform pose0, pose1;
  tf::poseMsgToTF(p0, pose0);
  tf::poseMsgToTF(p1, pose1);
  double sum = 0;

  BOOST_FOREACH (const Correspondence& c, corresp) 
  {
    const tf::Vector3 pt0(c.first->getPosition().x, c.first->getPosition().y, 0.0);
    const tf::Vector3 pt1(c.second->getPosition().x, c.second->getPosition().y, 0.0);
    sum += (pose0*pt0).distance(pose1*pt1);
  }
  return sum/n;
}

/************************************************************
 * Node class
 ***********************************************************/

class Node
{
public:
  Node ();

  void mainLoop (const ros::TimerEvent& e);
  void scanCB (unsigned i, const sm::LaserScan::ConstPtr& scan);

private:

  // Needed during initialization
  boost::mutex mutex_;
  ros::NodeHandle nh_;

  // State
  vector<sm::LaserScan::ConstPtr> scan_;

  // Flirtlib objects
  boost::shared_ptr<SimpleMinMaxPeakFinder> peak_finder_;
  boost::shared_ptr<HistogramDistance<double> > histogram_dist_;
  boost::shared_ptr<Detector> detector_;
  boost::shared_ptr<DescriptorGenerator> descriptor_;
  boost::shared_ptr<RansacFeatureSetMatcher> ransac_;
  OrientedPoint2D global_transform_;
  
  // Ros objects
  ros::Subscriber scan_subs_[2];
  ros::Publisher marker_pub_;
  ros::Publisher scan_pub_[2];
  tf::TransformListener tf_;
  ros::Timer exec_timer_;
  vector<gm::Pose> global_transforms_;
};


/************************************************************
 * Initialization
 ***********************************************************/


SimpleMinMaxPeakFinder* createPeakFinder ()
{
  return new SimpleMinMaxPeakFinder(0.34, 0.001);
}

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
  const double max_rho = 1.;//0.5;
  const double bin_rho = 4;
  const double bin_phi = 12;
  BetaGridGenerator* gen = new BetaGridGenerator(min_rho, max_rho, bin_rho,
                                                 bin_phi);
  gen->setDistanceFunction(dist);
  return gen;
}

Node::Node () :
  scan_(2), peak_finder_(createPeakFinder()),
  histogram_dist_(new SymmetricChi2Distance<double>()),
  detector_(createDetector(peak_finder_.get())),
  descriptor_(createDescriptor(histogram_dist_.get())),
  ransac_(new RansacFeatureSetMatcher(0.0599, 0.95, 0.1, 0.4,
                                      0.0384, /*true*/false)),

  marker_pub_(nh_.advertise<vm::Marker>("visualization_marker", 10)),
  exec_timer_(nh_.createTimer(ros::Duration(0.01), &Node::mainLoop, this))
{
  for (unsigned i=0; i<2; i++)
  {
    const string topic = "scan" + boost::lexical_cast<string>(i);
    scan_subs_[i] = nh_.subscribe<sm::LaserScan>(topic, 10,
                                                 boost::bind(&Node::scanCB, this, i, _1));
  }
}



/************************************************************
 * Visualization
 ***********************************************************/


// Generate markers to visualize correspondences between two scans
vm::Marker correspondenceMarkers (const Correspondences& correspondences,
                                  const gm::Pose& p0, const gm::Pose& p1)
{
  vm::Marker m;
  m.header.frame_id = "/base_laser_link";
  m.header.stamp = ros::Time::now();
  m.ns = "flirtlib";
  m.id = 42;
  m.type = vm::Marker::LINE_LIST;
  m.scale.x = 0.05;
  m.color.r = m.color.a = 1.0;
  m.color.g = 0.65;
  tf::Pose pose0, pose1;
  tf::poseMsgToTF(p0, pose0);
  tf::poseMsgToTF(p1, pose1);

  BOOST_FOREACH (const Correspondence& c, correspondences) 
  {
    const tf::Vector3 pt0(c.first->getPosition().x, c.first->getPosition().y, 0.0);
    const tf::Vector3 pt1(c.second->getPosition().x, c.second->getPosition().y, 0.0);
    m.points.push_back(toPoint(pose0*pt0));
    m.points.push_back(toPoint(pose1*pt1));
  }

  return m;
}


/************************************************************
 * Callbacks
 ***********************************************************/

void Node::scanCB (const unsigned i, const sm::LaserScan::ConstPtr& scan)
{
  Lock l(mutex_);
//  scan_[i] = scan;
  scan_[1] = scan_[0];
  scan_[0] = scan; 

  
}


/************************************************************
 * Main loop
 ***********************************************************/


void Node::mainLoop (const ros::TimerEvent& e)
{
  Lock l(mutex_);
  if (!scan_[0] || !scan_[1])
  {
    ROS_INFO_THROTTLE (1.0, "Waiting till scans received");
    return;
  }
    
  try
  {
    gm::Pose pose[2];
    InterestPointVec pts[2];
    boost::shared_ptr<LaserReading> reading[2];
    
    tf::Pose origin;
    origin.setIdentity();
    poseTFToMsg(origin, pose[0]);
    poseTFToMsg(origin, pose[1]);

    for (unsigned i=0; i<2; i++)
    {
      // Get the poses
      // This part can throw a tf exception
//      tf::StampedTransform trans;
//      const string frame = "pose"+boost::lexical_cast<string>(i);
//      tf_.lookupTransform("/map", frame, ros::Time(), trans);
//      tf::poseTFToMsg(trans, pose[i]);

      // Convert scans to flirtlib
      reading[i] = fromRos(*scan_[i]);

      // Interest point detection
      detector_->detect(*reading[i], pts[i]);
      marker_pub_.publish(interestPointMarkers(pts[i], pose[i], i));

      // Descriptor computation
      BOOST_FOREACH (InterestPoint* p, pts[i]) 
        p->setDescriptor(descriptor_->describe(*p, *reading[i]));
    }

    // Feature matching
    Correspondences matches;
    OrientedPoint2D transform;
    const double score = ransac_->matchSets(pts[0], pts[1], transform, matches);
    gm::Pose transf_pose = toRos(transform);

    global_transform_ = global_transform_.oplus(transform);
    global_transforms_.push_back(toRos(global_transform_));

    ROS_INFO_THROTTLE (0.5, "Found %zu matches, with score %.4f, transform "
                       "(%.2f, %.2f, %.2f), avg dist %.2f", matches.size(),
                       score, transform.x, transform.y, transform.theta,
                       averageDistance(matches, transf_pose, pose[0]));

    marker_pub_.publish(correspondenceMarkers(matches, pose[1], pose[0]));

    marker_pub_.publish(poseMarkers(global_transforms_).back());


    // Free memory
    for (unsigned i=0; i<2; i++)
    {
      BOOST_FOREACH (const InterestPoint* p, pts[i]) 
        delete p;
    }
  }
  catch (tf::TransformException& e)
  {
    ROS_INFO("Skipping frame due to transform exception");
  }    
}



} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "flirtlib_ros_test");
  flirtlib_ros::Node node;
  ros::spin();
  return 0;
}
