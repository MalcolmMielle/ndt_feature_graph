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

#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <tf/transform_broadcaster.h>

namespace flirtlib_ros
{

namespace gm=geometry_msgs;
namespace im=interactive_markers;
namespace vm=visualization_msgs;

using std::vector;
using std::string;
using boost::bind;
using boost::optional;

typedef boost::mutex::scoped_lock Lock;


/************************************************************
 * Node class
 ***********************************************************/

class MarkerNode 
{
public:

  MarkerNode ();
  void publishPoses (const ros::TimerEvent& e);
  void handlePoseUpdate (const unsigned id, const vm::InteractiveMarkerFeedback::ConstPtr& f);

private:

  ros::NodeHandle nh_;
  const double pub_rate_;
  const double num_markers_;

  vector<gm::Pose> poses_;

  boost::mutex mutex_;
  ros::Timer pub_timer_;
  tf::TransformBroadcaster tfb_;
  im::InteractiveMarkerServer im_server_;
};

/************************************************************
 * Functions
 ***********************************************************/

template <typename T>
T getParam (const string& name, const T& default_val)
{
  T val;
  ros::NodeHandle nh("~");
  nh.param(name, val, default_val);
  ROS_INFO_STREAM ("Setting " << name << " to " << val);
  return val;
}

ros::Duration duration (const double r)
{
  return ros::Duration(r==0.0 ? 0.0 : 1/r);
}


MarkerNode::MarkerNode () :
  pub_rate_(getParam<double>("pub_rate", 10.0)),
  num_markers_(getParam<int>("num_markers", 2)),
  pub_timer_(nh_.createTimer(duration(pub_rate_),
                             &MarkerNode::publishPoses, this)),
  im_server_("int_markers", "", false)
{
  poses_.resize(num_markers_);
  for (unsigned i=0; i<num_markers_; i++)
  {
    // Set up an interactive marker that allows moving marker in 2d
    vm::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/map";
    int_marker.scale = 0.5;
    int_marker.pose.position.y = 25.0+2.0*i;
    int_marker.pose.position.x = 15.0;
    int_marker.pose.orientation.w = 1.0;
    poses_[i] = int_marker.pose;

    int_marker.name = string("marker") + boost::lexical_cast<string>(i);

    vm::Marker marker;
    marker.type = vm::Marker::ARROW;
    marker.scale.x = 2.0;
    marker.scale.y = 2.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    vm::InteractiveMarkerControl control;
    control.orientation.w = 1.0;
    control.orientation.y = 1.0;
    control.interaction_mode = vm::InteractiveMarkerControl::MOVE_ROTATE;
    control.always_visible = true;
    control.markers.push_back(marker);
    int_marker.controls.push_back(control);

    im_server_.insert(int_marker);
    im_server_.setCallback(int_marker.name,
                           bind(&MarkerNode::handlePoseUpdate,
                                this, i, _1),
                           vm::InteractiveMarkerFeedback::POSE_UPDATE);

    
  }
  im_server_.applyChanges();
  ROS_INFO ("Initialized");
}


tf::StampedTransform toTf (const gm::Pose& p, const string& frame)
{
  tf::Transform trans;
  const gm::Point pos=p.position;
  const gm::Quaternion rot=p.orientation;
  trans.setOrigin(btVector3(pos.x, pos.y, pos.z));
  trans.setRotation(btQuaternion(rot.x, rot.y, rot.z, rot.w));
  return tf::StampedTransform(trans, ros::Time::now(), "/map", frame);
}

void MarkerNode::publishPoses (const ros::TimerEvent& e)
{
  Lock l(mutex_);
  if (poses_.size()>=num_markers_)
  {
    for (unsigned i=0; i<num_markers_; i++)
    {
      const string frame = "pose" + boost::lexical_cast<string>(i);
      tfb_.sendTransform(toTf(poses_[i], frame));
    }
  }
  else
    ROS_INFO_THROTTLE(2.0, "Waiting for marker poses before publishing");
}

void MarkerNode::handlePoseUpdate
(const unsigned i,
 const vm::InteractiveMarkerFeedback::ConstPtr& f)
{
  Lock l(mutex_);
  ROS_DEBUG_STREAM_NAMED ("update", "Received update for marker " << i <<
                          "; pose is now " << f->pose);
  poses_[i] = f->pose;
}




} // namespace


int main (int argc, char** argv)
{
  ros::init(argc, argv, "interactive_markers");
  flirtlib_ros::MarkerNode node;
  ros::spin();
  return 0;
}
