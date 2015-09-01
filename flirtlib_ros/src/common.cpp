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

#include <flirtlib_ros/common.h>

namespace flirtlib_ros
{

gm::Pose getCurrentPose (const tf::TransformListener& tf,
                         const std::string& frame)
{
  tf::StampedTransform trans;
  tf.lookupTransform("/map", frame, ros::Time(), trans);
  gm::Pose pose;
  tf::poseTFToMsg(trans, pose);
  return pose;
}


gm::Pose makePose (const double x, const double y, const double theta)
{
  gm::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.orientation = tf::createQuaternionMsgFromYaw(theta);
  return p;
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

tf::Transform getLaserOffset (const tf::TransformListener& tf)
{
  const std::string LASER_FRAME = "base_laser_link";
  const std::string BASE_FRAME = "base_footprint";
  while (ros::ok())
  {
    tf.waitForTransform(LASER_FRAME, BASE_FRAME, ros::Time(),
                        ros::Duration(5.0));
    try {
      tf::StampedTransform trans;
      tf.lookupTransform(LASER_FRAME, BASE_FRAME, ros::Time(),
                         trans);
      return trans;
    }
    catch (tf::TransformException& e)
    {
      ROS_INFO ("Waiting for transform between %s and %s",
                LASER_FRAME.c_str(), BASE_FRAME.c_str());
    }
  }
  return tf::Transform(); // Should never happen
}

} // namespace
