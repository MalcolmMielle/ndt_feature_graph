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
#include <mongo_ros/message_collection.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>

namespace flirtlib_ros
{

namespace sm=sensor_msgs;
namespace vm=visualization_msgs;
namespace gm=geometry_msgs;
namespace mr=mongo_ros;

using std::string;
using std::vector;

typedef boost::mutex::scoped_lock Lock;
typedef vector<InterestPoint*> InterestPointVec;
typedef std::pair<InterestPoint*, InterestPoint*> Correspondence;
typedef vector<Correspondence> Correspondences;
typedef vector<RefScan> RefScans;



/************************************************************
 * Node class
 ***********************************************************/

class Node
{
public:

  Node ();
  void scanCB (sm::LaserScan::ConstPtr scan);

private:

  void initializeRefScans();
  gm::Pose getPose();

  // Needed during initialization
  boost::mutex mutex_;
  ros::NodeHandle nh_;

  // Parameters
  const double min_num_matches_;
  const tf::Transform laser_offset_;

  // State
  RefScans ref_scans_;

  // Flirtlib objects
  boost::shared_ptr<SimpleMinMaxPeakFinder> peak_finder_;
  boost::shared_ptr<HistogramDistance<double> > histogram_dist_;
  boost::shared_ptr<Detector> detector_;
  boost::shared_ptr<DescriptorGenerator> descriptor_;
  boost::shared_ptr<RansacFeatureSetMatcher> ransac_;
  
  // Ros objects
  tf::TransformListener tf_;
  ros::Subscriber scan_sub_;
  ros::Publisher marker_pub_;
  ros::Publisher ref_scan_pose_pub_;
  ros::Publisher match_pose_pub_;
  ros::Publisher adjusted_pose_pub_;
  ros::Publisher pose_est_pub_;
  mr::MessageCollection<RefScanRos> scans_;

};


/************************************************************
 * Initialization
 ***********************************************************/

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

template <class T>
T getPrivateParam (const string& name, const T& default_val)
{
  ros::NodeHandle nh("~");
  T val;
  nh.param(name, val, default_val);
  ROS_DEBUG_STREAM_NAMED ("init", "Initialized " << name << " to " << val <<
                          "(default was " << default_val << ")");
  return val;
}

SimpleMinMaxPeakFinder* createPeakFinder ()
{
  return new SimpleMinMaxPeakFinder(0.34, 0.001);
}

Detector* createDetector (SimpleMinMaxPeakFinder* peak_finder)
{
  const double scale = 7.0;
  const double dmst = 2.0;
  const double base_sigma = 0.1;
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

tf::Transform loadLaserOffset ()
{
  const double yaw = getPrivateParam<double>("laser_offset_yaw", 0.0);
  const double x = getPrivateParam<double>("laser_offset_x", 0.0);
  const double y = getPrivateParam<double>("laser_offset_y", 0.0);
  return tf::Transform(tf::createQuaternionFromYaw(yaw), tf::Vector3(x, y, 0));
}

Node::Node () :

  min_num_matches_(getPrivateParam<int>("min_num_matches", 10)),
  laser_offset_(loadLaserOffset()),
  
  peak_finder_(createPeakFinder()),
  histogram_dist_(new SymmetricChi2Distance<double>()),
  detector_(createDetector(peak_finder_.get())),
  descriptor_(createDescriptor(histogram_dist_.get())),
  ransac_(new RansacFeatureSetMatcher(0.0599, 0.98, 0.4, 0.4,
                                           0.0384, false)),
  scan_sub_(nh_.subscribe("scan", 1, &Node::scanCB, this)),
  marker_pub_(nh_.advertise<vm::Marker>("visualization_marker", 10)),
  ref_scan_pose_pub_(nh_.advertise<gm::PoseArray>("ref_scan_poses", 10, true)),
  match_pose_pub_(nh_.advertise<gm::PoseArray>("match_poses", 1)),
  adjusted_pose_pub_(nh_.advertise<gm::PoseArray>("adjusted_poses", 1)),
  pose_est_pub_(nh_.advertise<gm::PoseStamped>("pose_estimate", 1)),
  scans_(getPrivateParam<string>("scan_db"), "scans")
{
  ROS_DEBUG_NAMED ("init", "Db contains %u scans.",
                   scans_.count());
  initializeRefScans();
  ROS_INFO ("Startup loc initialized");
}

gm::Pose makePose (const double x, const double y, const double theta)
{
  gm::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.orientation = tf::createQuaternionMsgFromYaw(theta);
  return p;
}

void Node::initializeRefScans ()
{
  gm::PoseArray poses;
  poses.header.stamp = ros::Time::now();
  poses.header.frame_id = "/map";
  unsigned i=0;
  
  BOOST_FOREACH (mr::MessageWithMetadata<RefScanRos>::ConstPtr m,
                 scans_.queryResults(mr::Query(), false)) 
  {
    ROS_DEBUG_NAMED("init", "Reading scan %u", i);
    ref_scans_.push_back(fromRos(*m));
    poses.poses.push_back(makePose(m->lookupDouble("x"), m->lookupDouble("y"),
                                   m->lookupDouble("theta")));
  }
  ref_scan_pose_pub_.publish(poses);
}


/************************************************************
 * Main
 ***********************************************************/


gm::Pose Node::getPose ()
{
  tf::StampedTransform trans;
  tf_.lookupTransform("/map", "base_laser_link", ros::Time(), trans);
  gm::Pose pose;
  tf::poseTFToMsg(trans, pose);
  return pose;
}

gm::Pose transformPose (const gm::Pose& p, const OrientedPoint2D& trans)
{
  tf::Transform laser_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.275, 0, 0));
  tf::Transform tr;
  tf::poseMsgToTF(p, tr);
  tf::Transform rel(tf::createQuaternionFromYaw(trans.theta),
                    tf::Vector3(trans.x, trans.y, 0.0));
  gm::Pose ret;
  tf::poseTFToMsg(tr*rel*laser_pose, ret);
  return ret;
}

gm::Pose transformPose (const tf::Transform& trans, const gm::Pose& pose)
{
  tf::Transform p;
  tf::poseMsgToTF(pose, p);
  gm::Pose ret;
  tf::poseTFToMsg(trans*p, ret);
  return ret;
}



void Node::scanCB (sm::LaserScan::ConstPtr scan)
{
  try
  {
    // Getting pose is the part that can throw exceptions
    const gm::Pose current_pose = getPose();
    const double theta = tf::getYaw(current_pose.orientation);
    const double x=current_pose.position.x;
    const double y=current_pose.position.y;

    // Extract features for this scan
    InterestPointVec pts;
    boost::shared_ptr<LaserReading> reading = fromRos(*scan);
    detector_->detect(*reading, pts);
    BOOST_FOREACH (InterestPoint* p, pts) 
      p->setDescriptor(descriptor_->describe(*p, *reading));
    marker_pub_.publish(interestPointMarkers(pts, current_pose, 0));

    // Match
    gm::PoseArray match_poses;
    int best_num_matches = -1;
    gm::PoseArray adjusted_poses;
    gm::PoseStamped best_pose;
    best_pose.header.frame_id = adjusted_poses.header.frame_id =
      match_poses.header.frame_id = "/map";
    best_pose.header.stamp = adjusted_poses.header.stamp =
      match_poses.header.stamp = ros::Time::now();
    ROS_DEBUG_NAMED ("match", "Matching scan at %.2f, %.2f, %.2f", x, y, theta);
    BOOST_FOREACH (const RefScan& ref_scan, ref_scans_) 
    {
      Correspondences matches;
      OrientedPoint2D trans;
      ransac_->matchSets(ref_scan.raw_pts, pts, trans, matches);
      const int num_matches = matches.size();
      if (num_matches > min_num_matches_)
      {
        ROS_DEBUG_NAMED ("match", "Found %d matches with ref scan at "
                         "%.2f, %.2f, %.2f", num_matches,
                         ref_scan.pose.position.x, ref_scan.pose.position.y,
                         tf::getYaw(ref_scan.pose.orientation));
        match_poses.poses.push_back(ref_scan.pose);
        const gm::Pose laser_pose = transformPose(ref_scan.pose, trans);
        const gm::Pose adjusted_pose = transformPose(laser_offset_, laser_pose);
        if (num_matches > best_num_matches)
        {
          best_num_matches = num_matches;
          ROS_DEBUG_NAMED ("match", "Transform is %.2f, %.2f, %.2f."
                           "  Transformed pose is %.2f, %.2f, %.2f",
                           trans.x, trans.y, trans.theta,
                           adjusted_pose.position.x, adjusted_pose.position.y,
                           tf::getYaw(adjusted_pose.orientation));
          best_pose.pose = adjusted_pose;
        }
      }
    }
    if (best_num_matches >= min_num_matches_)
    {
      match_pose_pub_.publish(match_poses);
      adjusted_pose_pub_.publish(adjusted_poses);
      pose_est_pub_.publish(best_pose);
    }
  }

  catch (tf::TransformException& e)
  {
    ROS_INFO ("Skipping because of tf exception");
  }
}


} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "startup_loc");
  flirtlib_ros::Node node;
  ros::spin();
  return 0;
}
