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

#ifndef FLIRTLIB_ROS_COMMON_H
#define FLIRTLIB_ROS_COMMON_H

#include <tf/transform_listener.h>
#include <flirtlib_ros/flirtlib.h>

namespace flirtlib_ros
{

namespace gm=geometry_msgs;

inline
gm::Pose tfTransformToPose (const tf::Transform& trans)
{
  gm::Pose m;
  tf::poseTFToMsg(trans, m);
  return m;
}

inline
tf::Pose poseMsgToTf (const gm::Pose& p)
{
  const gm::Quaternion& q = p.orientation;
  const gm::Point& v = p.position;
  return tf::Pose(tf::Quaternion(q.x, q.y, q.z, q.w),
                  tf::Vector3(v.x, v.y, v.z));
}


template <typename T>
T getPrivateParam (const std::string& name, const T& default_val)
{
  ros::NodeHandle nh("~");
  T val;
  nh.param(name, val, default_val);
  ROS_DEBUG_STREAM_NAMED ("init", "Initialized " << name << " to " << val <<
                          "(default was " << default_val << ")");
  return val;
}
  


gm::Pose getCurrentPose (const tf::TransformListener& tf,
                         const std::string& frame);

gm::Pose transformPose (const gm::Pose& p, const OrientedPoint2D& trans);

gm::Pose transformPose (const tf::Transform& trans, const gm::Pose& pose);

gm::Pose makePose (double x, double y, double theta);

tf::Transform getLaserOffset (const tf::TransformListener& tf);

} // namespace

#endif // include guard
