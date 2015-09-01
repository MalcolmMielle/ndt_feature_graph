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

#include <flirtlib_ros/localization_monitor.h>
#include <flirtlib_ros/conversions.h>
#include <flirtlib_ros/common.h>
#include <flirtlib_ros/ExecutiveState.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/exceptions.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mongo_ros/message_collection.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <std_srvs/Empty.h>


namespace flirtlib_ros
{

namespace sm=sensor_msgs;
namespace gm=geometry_msgs;
namespace nm=nav_msgs;
namespace mr=mongo_ros;
namespace vm=visualization_msgs;
namespace mbm=move_base_msgs;

using std::string;
using std::vector;
using boost::format;
using boost::shared_ptr;

typedef boost::mutex::scoped_lock Lock;
typedef mr::MessageWithMetadata<RefScanRos>::ConstPtr DBScan;
typedef std::pair<InterestPoint*, InterestPoint*> Correspondence;
typedef vector<Correspondence> Correspondences;
typedef vector<RefScan> RefScans;
typedef mr::MessageWithMetadata<nm::OccupancyGrid> MapWithMetadata;

class Node
{
public:

  Node ();

  // Update given new laser scan.
  void scanCB (sm::LaserScan::ConstPtr scan);
  
  // Save the map the first time it comes in
  void mapCB (const nm::OccupancyGrid& g);
  
  // Used to count successful navigations, as an indicator that we're
  // well localized.
  void navCB (const mbm::MoveBaseActionResult& m);
  
  // Publish the executive state periodically
  void publishExecState (const ros::TimerEvent& e);
  
  // Reset the executive state
  bool resetExecState (std_srvs::Empty::Request& req,
                       std_srvs::Empty::Response& resp);
  
private:
  
  // Update given that we're well localized
  void updateLocalized (sm::LaserScan::ConstPtr scan, const gm::Pose& p);
  
  // Update given that we're not well localized
  void updateUnlocalized (sm::LaserScan::ConstPtr scan);
  
  // Republish the set of reference scan poses.
  void publishRefScans () const;
  
  // Get saved occupancy grid from db
  MapWithMetadata::ConstPtr getSavedGrid() const;
  
  // Compensate for base movement between when the scan was taken and now
  tf::Transform compensateOdometry (const tf::Pose& sensor_pose,
                                    const string& frame, const ros::Time& t1,
                                    const ros::Time& t2);
  
  /************************************************************
   * Needed during init
   ************************************************************/
  
  boost::mutex mutex_;
  ros::NodeHandle nh_;
  
  /************************************************************
   * Parameters
   ************************************************************/ 

  // Minimum number of feature matches needed for succesful scan match
  unsigned min_num_matches_; 
  
  // Minimum number of navigations to happen before we'll save new scans
  unsigned min_successful_navs_;
  
  // Name of the database
  string db_name_;
  
  // Host where the db is running
  string db_host_;
  
  // Rate at which we'll consider new scans
  ros::Rate update_rate_;
  
  // Bound on badness to consider the robot well localized
  double badness_threshold_;
  
  // Odometric frame
  string odom_frame_;
  
  // Whether we continually publish localizations on initialpose 
  // (if false, we'll just publish one until reset)
  bool continual_publish_;
  
  /************************************************************
   * Mutable state
   ************************************************************/
  
  // Local copy of db scans. 
  vector<RefScan> ref_scans_;
  
  // Offset between laser and base frames
  tf::Transform laser_offset_;
  
  // Will we publish a localization next time we find a good match
  bool publishing_loc_;
  
  // How many successful navigations have we observed
  unsigned successful_navs_;
  
  // The last measurement of localization badness
  double localization_badness_;
  
  /************************************************************
   * Associated objects
   ************************************************************/
  
  // Flirtlib detector and descriptor
  FlirtlibFeatures features_;

  // Evaluates localization badness based on the scan and the static map
  shared_ptr<ScanPoseEvaluator> evaluator_;
  
  // Db collection of scans
  mr::MessageCollection<RefScanRos> scans_;
  
  tf::TransformListener tf_;
  ros::Subscriber scan_sub_;
  ros::Subscriber map_sub_;
  ros::Publisher marker_pub_;
  ros::Publisher ref_scan_pose_pub_;
  ros::Publisher pose_est_pub_;
  ros::Publisher match_pose_pub_;
  ros::Publisher exec_state_pub_;
  ros::Publisher pose_pub_;
  ros::Subscriber nav_sub_;
  ros::NodeHandle pnh_;
  ros::ServiceServer reset_srv_;
  ros::Timer pub_timer_;
};

// Reset to the executive state upon startup
bool Node::resetExecState (std_srvs::Empty::Request& req,
                           std_srvs::Empty::Response& resp)
{
  Lock l(mutex_);
  publishing_loc_ = true;
  successful_navs_ = 0;
  localization_badness_ = -1;
  return true;
}


// Constructor sets up ros comms and flirtlib objects, loads scans from db,
// and figures out the offset between the base and the laser frames
Node::Node () :
  min_num_matches_(getPrivateParam<int>("min_num_matches", 8)),
  min_successful_navs_(getPrivateParam<int>("min_successful_navs", 1)),
  db_name_(getPrivateParam<string>("db_name", "flirtlib_place_rec")),
  db_host_(getPrivateParam<string>("db_host", "localhost")),
  update_rate_(getPrivateParam<double>("update_rate", 1.0)),
  badness_threshold_(0.25),
  odom_frame_(getPrivateParam<string>("odom_frame", "odom_combined")),
  continual_publish_(getPrivateParam<bool>("continual_publish", false)),
  publishing_loc_(true),
  successful_navs_(0), localization_badness_(-1),
  features_(ros::NodeHandle("~")), scans_(db_name_, "scans", db_host_),
  scan_sub_(nh_.subscribe("base_scan", 10, &Node::scanCB, this)),
  map_sub_(nh_.subscribe("map", 10, &Node::mapCB, this)),
  marker_pub_(nh_.advertise<vm::Marker>("visualization_marker", 10)),
  ref_scan_pose_pub_(nh_.advertise<gm::PoseArray>("ref_scan_poses", 10, true)),
  pose_est_pub_(nh_.advertise<gm::PoseStamped>("pose_estimate", 1)),
  match_pose_pub_(nh_.advertise<gm::PoseArray>("match_poses", 1)),
  exec_state_pub_(nh_.advertise<ExecutiveState>("startup_loc_state", 1)),
  pose_pub_(nh_.advertise<gm::PoseWithCovarianceStamped>("initialpose", 1)),
  nav_sub_(nh_.subscribe("move_base/result", 1, &Node::navCB, this)), pnh_("~"),
  reset_srv_(pnh_.advertiseService("reset_state", &Node::resetExecState, this)),
  pub_timer_(nh_.createTimer(ros::Duration(1.0), &Node::publishExecState, this))
{
  ROS_DEBUG_NAMED("init", "Waiting for laser offset");
  laser_offset_ = getLaserOffset(tf_);

  ROS_DEBUG_NAMED("init", "Loading scans from db");
  BOOST_FOREACH (const mr::MessageWithMetadata<RefScanRos>::ConstPtr m,
                 scans_.queryResults(mr::Query(), false)) 
    ref_scans_.push_back(fromRos(*m));
  publishRefScans();
  ROS_INFO("Localization monitor initialized with %zu scans", ref_scans_.size());
}

// Publish the executive state
void Node::publishExecState (const ros::TimerEvent& e)
{
  Lock l(mutex_);
  ExecutiveState s;
  s.publishing_localization = publishing_loc_;
  s.successful_navs = successful_navs_;
  s.localization_badness = localization_badness_;
  exec_state_pub_.publish(s);
}

// Update counter of successful navs observed
void Node::navCB (const mbm::MoveBaseActionResult& res)
{
  if (res.status.status == actionlib_msgs::GoalStatus::SUCCEEDED)
  {
    successful_navs_++;
    if (successful_navs_==min_successful_navs_)
    {
      ROS_INFO ("Have now observed %u successful navigations.  Can "
                "now start saving scans", successful_navs_);
    }
  }
}

// Publish the poses of the reference scans (for visualization)
void Node::publishRefScans () const
{
  gm::PoseArray poses;
  poses.header.stamp = ros::Time::now();
  poses.header.frame_id = "/map";
  BOOST_FOREACH (const RefScan& s, ref_scans_) 
    poses.poses.push_back(s.pose);
  ref_scan_pose_pub_.publish(poses);
}

unsigned getSum (const nm::OccupancyGrid& g)
{
  unsigned sum=0;
  BOOST_FOREACH (const unsigned val, g.data) 
    sum += val;
  return sum;
}


// Use the map to initialize the localization evaluator
void Node::mapCB (const nm::OccupancyGrid& g)
{
  const unsigned my_sum = getSum(g);
  mr::MessageCollection<nm::OccupancyGrid> coll(db_name_, "grid", db_host_);
  ROS_INFO("Received map");
  if (coll.count()>0)
  {
    MapWithMetadata::ConstPtr saved_map =
      coll.pullAllResults(mr::Query(), true)[0];
    const double res = saved_map->lookupDouble("resolution");
    const unsigned height = saved_map->lookupInt("height");
    const unsigned width = saved_map->lookupInt("width");
    const unsigned sum = saved_map->lookupInt("sum");
    if (my_sum!=sum || height!=g.info.height || width!=g.info.width ||
        fabs(res-g.info.resolution)>1e-3)
    {
      ROS_FATAL("Db map is %ux%u at %.4f m/cell, with sum %u, which "
                "doesn't match current map of size %ux%u at %.4f m/cell"
                " with sum %u", height, width, res, sum, g.info.height,
                g.info.width, g.info.resolution, my_sum);
      ROS_BREAK();
    }
  }
  else
  {
    ROS_WARN("Didn't find an OccupancyGrid in the database.  Assuming this is "
             "a new db and saving current map with size %ux%u and sum %u",
             g.info.height, g.info.width, my_sum);
    mr::Metadata m = mr::Metadata().\
append("height", g.info.height).append("width", g.info.width).\
append("resolution", g.info.resolution).append("sum", my_sum);
    coll.insert(g, m);
  }
  ROS_INFO("Initializing scan evaluator distance field");
  Lock l(mutex_);
  evaluator_.reset(new ScanPoseEvaluator(g, badness_threshold_*1.5));
  ROS_INFO("Scan evaluator initialized");
}



// If we're not well localized, try to localize by matching against the scans in the db
  void Node::updateUnlocalized (sm::LaserScan::ConstPtr scan)
  {
    // Extract features from curent scan
    gm::Pose current = getCurrentPose(tf_, "base_footprint");
    ROS_INFO("Not well localized");
    InterestPointVec pts = features_.extractFeatures(scan);
    marker_pub_.publish(interestPointMarkers(pts, current));
      
    // Set up 
    gm::PoseArray match_poses;
    match_poses.header.frame_id = "/map";
    match_poses.header.stamp = ros::Time::now();
    unsigned best_num_matches = 0;
    gm::Pose best_pose;
  
    // Iterate over reference scans and match against each one, looking for the
    // one with the largest number of feature matches
    BOOST_FOREACH (const RefScan& ref_scan, ref_scans_) 
    {
      Correspondences matches;
      OrientedPoint2D trans;
      features_.ransac_->matchSets(ref_scan.raw_pts, pts, trans, matches);
      const unsigned num_matches = matches.size();
      if (num_matches > min_num_matches_) 
      {
        ROS_INFO_NAMED ("match", "Found %d matches with ref scan at "
                        "%.2f, %.2f, %.2f", num_matches,
                        ref_scan.pose.position.x, ref_scan.pose.position.y,
                        tf::getYaw(ref_scan.pose.orientation));
        match_poses.poses.push_back(ref_scan.pose);
        const gm::Pose laser_pose = transformPose(ref_scan.pose, trans);
        if (num_matches > best_num_matches)
        {
          best_num_matches = num_matches;
          best_pose = laser_pose;
        }
      }
    }

    // Only proceed if there are a sufficient number of feature matches
    if (best_num_matches<min_num_matches_)
      return;

    match_pose_pub_.publish(match_poses);
    const double badness = (*evaluator_)(*scan, best_pose);
    ROS_INFO ("Badness is %.2f", badness);
  
    // Do a further check of scan badness before committing to this
    // We'll always publish a visualization, but we'll only publish on the
    // initialpose topic once (so after that, this node will go into a state where
    // all it does is possibly save new scans).
    if (badness < badness_threshold_)
    {
      ROS_INFO("Found a good match");
    
      const ros::Time now = ros::Time::now();
      gm::PoseStamped estimated_pose;
      try {
        tf::Pose adjusted_pose(compensateOdometry(poseMsgToTf(best_pose),
                                                  scan->header.frame_id,
                                                  scan->header.stamp, now));
        tf::poseTFToMsg(adjusted_pose*laser_offset_, estimated_pose.pose);
      }
      catch (tf::TransformException& e)
      {
        ROS_INFO ("Discarding match due to tf exception: %s", e.what());
        return;
      }
    

      estimated_pose.header.frame_id = "/map";
      estimated_pose.header.stamp = now;
      pose_est_pub_.publish(estimated_pose);
    
      if (!continual_publish_)
        publishing_loc_ = false;
      gm::PoseWithCovarianceStamped initial_pose;
      initial_pose.header.frame_id = "/map";
      initial_pose.header.stamp = scan->header.stamp;
      initial_pose.pose.pose = estimated_pose.pose;
      pose_pub_.publish(initial_pose);
    }
  }

tf::Transform Node::compensateOdometry (const tf::Pose& pose,
                                        const string& frame,
                                        const ros::Time& t1,
                                        const ros::Time& t2)
{
  tf_.waitForTransform(odom_frame_, frame, t1, ros::Duration(0.1));
  tf_.waitForTransform(odom_frame_, frame, t2, ros::Duration(0.1));

  tf::StampedTransform current_odom, prev_odom;
  tf_.lookupTransform(odom_frame_, frame, t2, current_odom);
  tf_.lookupTransform(odom_frame_, frame, t1, prev_odom);
  tf::Transform current_to_prev = prev_odom.inverse()*current_odom;
  return pose*current_to_prev;
}
                                        

// Search the db for a nearby scan.  If none is found, and also we've
// successfully navigated previously (as a further cue that we're well
// localized), then add this scan to the db, and remove any
// nearby old scans.
void Node::updateLocalized (sm::LaserScan::ConstPtr scan,
                            const gm::Pose& current)
{
  ROS_INFO("Well localized");
  const double DPOS = 0.7;
  const double DTHETA = 1;
  const double DT = 86400; // one day
  
  // Query the db for nearby scans
  const double t = ros::Time::now().toSec();
  const double x = current.position.x;
  const double x0 = x - DPOS;
  const double x1 = x + DPOS;
  const double y = current.position.y;
  const double y0 = y - DPOS;
  const double y1 = y + DPOS;
  const double th = tf::getYaw(current.orientation);
  const double th0 = th - DTHETA;
  const double th1 = th + DTHETA;
  const double min_time = t-DT;
  format f("{x : {$gt: %.4f, $lt: %.4f}, y : {$gt: %.4f, $lt: %.4f}, "
           "theta: {$gt: %.4f, $lt: %.4f}, creation_time: {$gt: %.8f} }");
  string str = (f % x0 % x1 % y0 % y1 % th0 % th1 % min_time).str();
  mongo::Query q = mongo::fromjson(str);
  vector<DBScan> scans = scans_.pullAllResults(q, true);
  
  // Possibly save this new scan
  if (scans.size()<1 && successful_navs_>=min_successful_navs_) 
  {
    // First remove old scans that are nearby
    format old("{x : {$gt: %.4f, $lt: %.4f}, y : {$gt: %.4f, $lt: %.4f}, "
               "theta: {$gt: %.4f, $lt: %.4f} }");
    const string old_query_str = (old % x0 % x1 % y0 % y1 % th0 % th1).str();
    const mongo::Query old_query = mongo::fromjson(old_query_str);
    const vector<DBScan> old_scans = scans_.pullAllResults(old_query, true);
    scans_.removeMessages(old_query);
    BOOST_FOREACH (const DBScan& old_scan, old_scans) 
    {
      ROS_INFO ("Removed old scan at (%.4f, %.4f, %.f) taken at %.4f",
                old_scan->lookupDouble("x"), old_scan->lookupDouble("y"),
                old_scan->lookupDouble("theta"),
                old_scan->lookupDouble("creation_time"));
    }

    // Now add this new scan
    InterestPointVec pts = features_.extractFeatures(scan);
    sm::LaserScan::Ptr stripped_scan(new sm::LaserScan(*scan));
    stripped_scan->ranges.clear();
    stripped_scan->intensities.clear();
    RefScan ref_scan(stripped_scan, current, pts);
    ref_scans_.push_back(ref_scan);
    scans_.insert(toRos(ref_scan),
                  mr::Metadata("x", x, "y", y, "theta", th));
    ROS_DEBUG_NAMED ("save_scan", "Saved scan at %.2f, %.2f, %.2f", x, y, th);
    
    publishRefScans();
  }
}


// Given current scan and pose, update state.  If well localized, we'll 
// consider adding this new scan to the db.  If not, we'll try to localize
// using the db.
void Node::scanCB (sm::LaserScan::ConstPtr scan)
{
  // First, look up the sensor pose at the time of the scan
  const std::string frame = scan->header.frame_id;
  tf_.waitForTransform("/map", frame, scan->header.stamp, ros::Duration(0.5));
  tf::StampedTransform trans;
  try
  {
    tf_.lookupTransform("/map", frame, scan->header.stamp, trans);
  }
  catch (tf::TransformException& e)
  {
    ROS_INFO_STREAM ("Skipping scan due to tf exception " << e.what());
    return;
  }
  
  // Have we initialized the evaluator yet?
  {
  Lock l(mutex_);
  if (!evaluator_)
  {
    ROS_INFO_STREAM ("Skipping scan as evaluator not yet initialized");
    return;
  }
    
  // Evaluate how well this scan is localized
  const gm::Pose pose = tfTransformToPose(trans);
  const double dist = (*evaluator_)(*scan, pose);
  localization_badness_ = dist;
  ROS_INFO ("Localization badness is %.2f", dist);
  
  if (dist < badness_threshold_)
    updateLocalized(scan, pose);
  else if (publishing_loc_)
    updateUnlocalized(scan);
  
  }
  // Make this callback happen at a bounded rate
  update_rate_.sleep();
}

} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "localization_monitor_node");
  flirtlib_ros::Node node;
  ros::spin();
  return 0;
}
