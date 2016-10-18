#ifndef NDT_FEATURE_RVIZ_H
#define NDT_FEATURE_RVIZ_H

#include <ndt_feature/interfaces.h>
#include <ndt_feature/ndt_rviz.h>
#include <ndt_feature/flirtlib_utils.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <ndt_feature/ndt_feature_frame.h>
#include <eigen_conversions/eigen_msg.h>

namespace ndt_feature {

/* geometry_msgs::Point toPoint (const tf::Vector3& p) */
/* { */
/*   geometry_msgs::Point pt; */
/*   pt.x = p.x(); */
/*   pt.y = p.y(); */
/*   pt.z = p.z(); */
/*   return pt; */
/* } */

// Generate markers to visualize correspondences between two scans
inline visualization_msgs::Marker correspondenceMarkers (const Correspondences& correspondences,
						  const geometry_msgs::Pose& ref, const geometry_msgs::Pose& curr,
						  const std::string &frame_id)
{
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = ros::Time::now();
  m.ns = "fl_corr";
  m.id = 42;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.scale.x = 0.05;
  m.color.r = m.color.a = 1.0;
  m.color.g = 0.65;
  tf::Pose pose_ref, pose_curr;
  tf::poseMsgToTF(ref, pose_ref);
  tf::poseMsgToTF(curr, pose_curr);

  BOOST_FOREACH (const Correspondence& c, correspondences) 
  {
    const tf::Vector3 curr_pt(c.first->getPosition().x, c.first->getPosition().y, 0.0); // Current
    const tf::Vector3 ref_pt(c.second->getPosition().x, c.second->getPosition().y, 0.0); // Ref
    //    std::cout << "pt0 : " << pt0 << "\t pt1 : " << pt1 << std::endl;
    m.points.push_back(ndt_visualisation::toPointFromTF(pose_curr*curr_pt));
    m.points.push_back(ndt_visualisation::toPointFromTF(pose_ref*ref_pt));
  }

  return m;
}

inline visualization_msgs::Marker correspondenceMarkers (const Correspondences& correspondences,
						  const std::string &frame_id)
{
  geometry_msgs::Pose pose;
  pose.position.x = pose.position.y = pose.position.z = 0.;
  pose.orientation.x = pose.orientation.y = pose.orientation.z = 0.;
  pose.orientation.w = 1.;
  
  return correspondenceMarkers(correspondences, pose, pose, frame_id);
}


// Generate visualization markers for the interest points
// id is 0 or 1, and controls color and orientation to distinguish between
// the two scans
inline visualization_msgs::Marker interestPointMarkersFrameId (const InterestPointVec& pts, const Eigen::Affine3d &T, const unsigned id, const std::string &frame_id)
{
  visualization_msgs::Marker m;
  ndt_visualisation::assignDefault(m);
  ndt_visualisation::assignColor(m,id);
  tf::Transform trans;
  tf::transformEigenToTF(T, trans);
  //  tf::poseMsgToTF(pose, trans);
  m.header.frame_id = frame_id;
  m.header.stamp = ros::Time::now();
  m.ns = "flirtlib";
  m.id = id;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.scale.x = 0.02;
  int i = 0; 
  BOOST_FOREACH (const InterestPoint* p, pts) 
  {
    const double x0 = p->getPosition().x;
    const double y0 = p->getPosition().y;
    const double d = 0.1;
    
    double dx[4];
    double dy[4];
    if (id==0)
    {
      dx[0] = dx[3] = -d;
      dx[1] = dx[2] = d;
      dy[0] = dy[1] = -d;
      dy[2] = dy[3] = d;
    }
    else
    {
      //      ROS_ASSERT(id==1);
      const double r2 = sqrt(2);
      dx[0] = dx[2] = dy[1] = dy[3] = 0;
      dx[1] = dy[0] = -r2*d;
      dx[3] = dy[2] = r2*d;
    }

    for (unsigned i=0; i<4; i++)
    {
      const unsigned j = (i==0) ? 3 : i-1;
      const tf::Point pt0(x0+dx[i], y0+dy[i], 0);
      const tf::Point pt1(x0+dx[j], y0+dy[j], 0);
      m.points.push_back(ndt_visualisation::toPointFromTF(trans*pt0));
      m.points.push_back(ndt_visualisation::toPointFromTF(trans*pt1));
    }
  }
    
  return m;
}

inline visualization_msgs::Marker interestPointMarkersFrameId (const InterestPointVec& pts, const geometry_msgs::Pose& pose, const unsigned id, const std::string &frame_id) {
  Eigen::Quaterniond qd;
    Eigen::Affine3d T;
    qd.x() = pose.orientation.x;
    qd.y() = pose.orientation.y;
    qd.z() = pose.orientation.z;
    qd.w() = pose.orientation.w;
    
    T = Eigen::Translation3d (pose.position.x,
                              pose.position.y,
                              pose.position.z) * qd;

    return interestPointMarkersFrameId(pts, T, id, frame_id);
}


inline visualization_msgs::Marker interestPointMarkersFrameId (const InterestPointVec& pts, const unsigned id, const std::string &frame_id)
{
  Eigen::Affine3d T;
  T.setIdentity();
  return interestPointMarkersFrameId (pts, T, id, frame_id);
}




inline visualization_msgs::Marker interestPointSupportMarkers (const InterestPointVec& pts, const geometry_msgs::Pose& pose, const unsigned id)
{
  tf::Transform trans;
  tf::poseMsgToTF(pose, trans);
  //  static ndt_visualisation::ColorVec colors = initColors();
  visualization_msgs::Marker m;
  m.header.frame_id = "/map";
  m.header.stamp = ros::Time::now();
  m.ns = "fl_support";
  m.id = id;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.scale.x = 0.005;
  //  m.color = colors[id];
  m.color.r = 0.2;
  m.color.b = 0.7;
  m.color.g = 0.7;
  m.color.a = 0.6;
  BOOST_FOREACH (const InterestPoint* p, pts) 
  {
    const double x0 = p->getPosition().x;
    const double y0 = p->getPosition().y;

    const std::vector< Point2D > &support = p->getSupport();
    BOOST_FOREACH(const Point2D sp, support)
    {
      const tf::Point pt0(x0, y0, 0);
      const tf::Point pt1(sp.x, sp.y, 0);
      m.points.push_back(ndt_visualisation::toPointFromTF(trans*pt0));
      m.points.push_back(ndt_visualisation::toPointFromTF(trans*pt1));
    }
  }
  return m;
}

inline visualization_msgs::Marker poseArrowMarker(const geometry_msgs::Pose& pose, const int id, const int color, const std::string &frame) {
  visualization_msgs::Marker m;
  ndt_visualisation::assignDefault(m);
  ndt_visualisation::assignColor(m,color);
  m.header.stamp = ros::Time::now();
  m.pose = pose;
  m.type = visualization_msgs::Marker::ARROW;
  m.ns = frame;
  m.id = id;
  m.scale.x = 1.0;
  m.scale.y = 0.2;
  m.scale.z = 0.1;
  return m;
}


inline visualization_msgs::Marker poseArrowMarkerEigen(const Eigen::Affine3d &pose, const int id, const int color, const std::string &frame) {
  geometry_msgs::Pose p;
  tf::poseEigenToMsg(pose, p);
  return poseArrowMarker(p, id, color, frame);
}



inline visualization_msgs::Marker posePointsMarkerNDTFeatureFrames(const std::vector<ndt_feature::NDTFeatureFrame> &frames, const int id, const int color, const std::string &frame, bool useOdom)  {
  visualization_msgs::Marker m;
  ndt_visualisation::assignDefault(m);
  ndt_visualisation::assignColor(m,color);
  m.type = visualization_msgs::Marker::POINTS;
  m.ns = frame;
  m.id = id;
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.color.a = 1;
  for (size_t i = 0; i < frames.size(); i++) {
    geometry_msgs::Pose pose;
    if (useOdom)
      tf::poseEigenToMsg(frames[i].odom, pose);
    else
      tf::poseEigenToMsg(frames[i].pose, pose);
    m.points.push_back(pose.position);
  }
  return m;
}

inline void publishMarkerNDTFeatureFrames(const std::vector<ndt_feature::NDTFeatureFrame> &frames, ros::Publisher &marker_pub) {
  
  int id = 0;
  BOOST_FOREACH (const ndt_feature::NDTFeatureFrame& f, frames) 
  {
    //marker_pub.publish(poseArrowMarkerEigen(f.odom, id, 0, std::string("frames_odom")));
    //marker_pub.publish(poseArrowMarkerEigen(f.gt, id, 1, std::string("frames_gt")));
    marker_pub.publish(poseArrowMarkerEigen(f.pose, id, 2, std::string("frames_pose")));
    id++;
  }
}


inline visualization_msgs::Marker featureMatchesMarkerNDTFeatureFrames(const std::vector<ndt_feature::NDTFeatureFrame> &frames,
								 const std::vector<std::pair<size_t, size_t> > &matches, 
								 const int id, const int color, const std::string &frame, bool useOdom) {
   // Draw a line between all the estimated poses
   visualization_msgs::Marker m;
   ndt_visualisation::assignDefault(m);
   ndt_visualisation::assignColor(m,color);
   m.ns = frame;
   m.id = id;
   m.type = visualization_msgs::Marker::LINE_LIST;
   m.scale.x = 0.02;
   m.color.r = m.color.a = 1.0;
   m.color.g = 0.65;
   
   geometry_msgs::Pose pose;
   for (size_t i = 0; i < matches.size(); i++) {
     geometry_msgs::Pose pose1, pose2;
     if (useOdom) {
       tf::poseEigenToMsg(frames[matches[i].first].odom, pose1);
       tf::poseEigenToMsg(frames[matches[i].second].odom, pose2);
     }
     else {
       tf::poseEigenToMsg(frames[matches[i].first].pose, pose1);
       tf::poseEigenToMsg(frames[matches[i].second].pose, pose2);
     }
     m.points.push_back(pose1.position);
     m.points.push_back(pose2.position);
   }
   
   return m;
     
 }
 
inline void publishMarkerNDTFeatureNodes(const NDTFeatureGraphInterface &graph, ros::Publisher &marker_pub) {
  
  int id = 0;
  for (size_t i = 0; i < graph.getNbNodes(); i++)
  {
    marker_pub.publish(poseArrowMarkerEigen(graph.getNodeInterface(i).getPose(), id, 2, std::string("nodes_pose")));
    id++;
  }
}

inline visualization_msgs::Marker markerNDTFeatureLinks(const NDTFeatureGraphInterface &graph, const int id, const int color, const std::string &frame, bool skipOdom) {
   visualization_msgs::Marker m;
   ndt_visualisation::assignDefault(m);
   ndt_visualisation::assignColor(m,color);
   m.ns = frame;
   m.id = id;
   m.type = visualization_msgs::Marker::LINE_LIST;
   m.scale.x = 0.02;
   m.color.r = m.color.a = 1.0;
   m.color.g = 0.65;
   
   geometry_msgs::Pose pose;
   for (size_t i = 0; i < graph.getNbLinks(); i++) {
     if (skipOdom) {
       if (graph.getLinkInterface(i).getScore() < 0) {
         continue;
       }
     }
     geometry_msgs::Pose pose1, pose2;
     size_t ref_idx = graph.getLinkInterface(i).getRefIdx();
     size_t mov_idx = graph.getLinkInterface(i).getMovIdx();
     
     tf::poseEigenToMsg(graph.getNodeInterface(ref_idx).getPose(), pose1);
     tf::poseEigenToMsg(graph.getNodeInterface(mov_idx).getPose(), pose2);
   
     m.points.push_back(pose1.position);
     m.points.push_back(pose2.position);
   }
   
   return m;
     
}

inline void publishMarkerNDTFeatureLinks(const NDTFeatureGraphInterface &graph, ros::Publisher &marker_pub) {
  marker_pub.publish(markerNDTFeatureLinks(graph, 0, 1, std::string("links"), true));
}


} // namespace

#endif
