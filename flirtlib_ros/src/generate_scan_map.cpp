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

#include <flirtlib_ros/conversions.h>
#include <mongo_ros/message_collection.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

namespace flirtlib_ros
{

namespace sm=sensor_msgs;
namespace gm=geometry_msgs;
namespace mr=mongo_ros;
namespace vm=visualization_msgs;
using std::vector;
using std::string;

typedef boost::mutex::scoped_lock Lock;
typedef vector<InterestPoint*> InterestPointVec;

class Node
{
public:
  
  Node();
  void scanCB (sm::LaserScan::ConstPtr scan);
  
private:

  bool haveNearbyScan (const gm::Pose& pose) const;
  gm::Pose getPoseAt (const ros::Time& t) const;
  RefScanRos extractFeatures (sm::LaserScan::ConstPtr scan,
                              const gm::Pose& pose);

  // Needed during init
  boost::mutex mutex_;
  ros::NodeHandle nh_;

  // Parameters
  const double pos_inc_, theta_inc_;

  // Flirtlib objects
  FlirtlibFeatures features_;
  
  // Ros objects
  tf::TransformListener tf_;
  ros::Subscriber scan_sub_;
  ros::Publisher marker_pub_;
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

Node::Node () :
  pos_inc_(getPrivateParam<double>("pos_inc")), 
  theta_inc_(getPrivateParam<double>("theta_inc")),
  features_(ros::NodeHandle("~")),
  scan_sub_(nh_.subscribe("scan", 1, &Node::scanCB, this)),
  marker_pub_(nh_.advertise<vm::Marker>("visualization_marker", 10)),
  scans_(getPrivateParam<string>("scan_db"), "scans")
{
  ROS_ASSERT_MSG(scans_.count()==0, "Scan collection was not empty");
  scans_.ensureIndex("x");
  ROS_INFO ("generate_scan_map initialized");
}

/************************************************************
 * Localization
 ***********************************************************/

inline
double angleDist (double t1, double t2)
{
  if (t2<t1)
    std::swap(t1,t2);
  const double d = t2-t1;
  return std::min(d, 6.2832-d);
}

bool Node::haveNearbyScan (const gm::Pose& pose) const
{
  using mongo::GT;
  using mongo::LT;
  
  bool found = false;
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double theta = tf::getYaw(pose.orientation);

  // Query for scans in a box around current pose
  mongo::Query q = BSON("x" << GT << x-pos_inc_ << LT << x+pos_inc_ <<
                        "y" << GT << y-pos_inc_ << LT << y+pos_inc_);
  BOOST_FOREACH (mr::MessageWithMetadata<RefScanRos>::ConstPtr scan,
                 scans_.queryResults(q, true))
  {
    // Also check angle distance
    if (angleDist(theta, scan->lookupDouble("theta")) < theta_inc_)
      found = true;
  }
  return found;
}

gm::Pose Node::getPoseAt (const ros::Time& t) const
{
  tf::StampedTransform trans;
  tf_.waitForTransform("/map", "base_laser_link", t, ros::Duration(0.5));
  tf_.lookupTransform("/map", "base_laser_link", t, trans);
  gm::Pose pose;
  tf::poseTFToMsg(trans, pose);
  return pose;
}

/************************************************************
 * Processing scans
 ***********************************************************/

RefScanRos Node::extractFeatures (sm::LaserScan::ConstPtr scan,
                                  const gm::Pose& pose)
{
  boost::shared_ptr<LaserReading> reading = fromRos(*scan);
  InterestPointVec pts;
  features_.detector_->detect(*reading, pts);
  BOOST_FOREACH (InterestPoint* p, pts) 
    p->setDescriptor(features_.descriptor_->describe(*p, *reading));
  marker_pub_.publish(interestPointMarkers(pts, pose));
  const RefScan ref(scan, pose, pts);
  return toRos(ref);
}


void Node::scanCB (sm::LaserScan::ConstPtr scan)
{
  try
  {
    const gm::Pose pose = getPoseAt(scan->header.stamp); // can throw
    if (haveNearbyScan(pose))
      return;

    // Now guaranteed no nearby pose
    const double x = pose.position.x;
    const double y = pose.position.y;
    const double th = tf::getYaw(pose.orientation);
    
    scans_.insert(extractFeatures(scan, pose),
                  mr::Metadata("x", x, "y", y, "theta", th));
    ROS_DEBUG_NAMED ("save_scan", "Saved scan at %.2f, %.2f, %.2f", x, y, th);
  }
  catch (tf::TransformException& e)
  {
    ROS_INFO ("Skipping callback due to tf error: '%s'", e.what());
  }
  
}

} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "generate_scan_map");
  flirtlib_ros::Node node;
  ros::spin();
  return 0;
}
