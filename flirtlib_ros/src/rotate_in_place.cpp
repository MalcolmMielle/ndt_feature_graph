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

#include <flirtlib_ros/RotateInPlaceAction.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <control_toolbox/pid.h>

namespace flirtlib_ros
{

namespace gm=geometry_msgs;
using std::string;

/************************************************************
 * Class def
 ***********************************************************/

class RotateInPlace
{
public:
  
  RotateInPlace();
  
private:

  control_toolbox::Pid pid_;

  ros::NodeHandle nh_, private_nh_;
  tf::TransformListener tf_;
  actionlib::SimpleActionServer<RotateInPlaceAction> as_;
  ros::Publisher vel_pub_;

  void execute (const RotateInPlaceGoal::ConstPtr& goal);
  double currentOrientation (const string& frame_id);
};


/************************************************************
 * Constructor
 ***********************************************************/

RotateInPlace::RotateInPlace () :
  private_nh_("~"),
  as_(nh_, "rotate_in_place", boost::bind(&RotateInPlace::execute, this, _1)),
  vel_pub_(nh_.advertise<gm::Twist>("navigation/cmd_vel", 1))
{
  pid_.init(private_nh_);
  double p, i, d, i_min, i_max;
  pid_.getGains(p, i, d, i_max, i_min);
  ROS_DEBUG_NAMED ("rotate_client", "Rotate action client started: (%.2f, %.2f, %.2f, %.2f, %.2f)",
                   p, i, d, i_min, i_max);
}


/************************************************************
 * Action server
 ***********************************************************/

double RotateInPlace::currentOrientation (const string& frame_id) 
{
  tf::StampedTransform trans;
  tf_.lookupTransform(frame_id, "base_footprint", ros::Time(), trans);
  return tf::getYaw(trans.getRotation());
}

void RotateInPlace::execute (const RotateInPlaceGoal::ConstPtr& goal)
{
  ROS_DEBUG_STREAM_NAMED ("rotate_client", "In the execute callback with goal " << *goal);
  RotateInPlaceResult result;
  ros::Duration d(0.1);
  pid_.updatePid(0, d); // for stability
  
  ROS_DEBUG_STREAM_NAMED ("rotate_client", "Waiting for transform");
  tf_.waitForTransform(goal->header.frame_id, "base_footprint", ros::Time::now(), ros::Duration(2.0));
  ROS_DEBUG_STREAM_NAMED ("rotate_client", "Found");
  
  while (ros::ok()) {
    const double current = currentOrientation(goal->header.frame_id);
    if (fabs(current-goal->target) < goal->tol) {
      result.achieved = current;
      break;
    }
    if (as_.isPreemptRequested()) 
    {
      result.achieved = current;
      as_.setPreempted();
      return;
    }
    gm::Twist vel;
    vel.angular.z = pid_.updatePid(current-goal->target, d);
    ROS_DEBUG_STREAM_THROTTLE_NAMED (1, "rotate_loop", "Current yaw is " << current << "; desired is " <<
                                     goal->target << "; pid effort is " << vel.angular.z);
    vel_pub_.publish(vel);
    d.sleep();
  }
  
  as_.setSucceeded(result);
}


} // namespace flirtlib_ros



int main (int argc, char** argv)
{
  ros::init(argc, argv, "rotate_in_place");
  flirtlib_ros::RotateInPlace node;
  ros::spin();
  return 0;
}
