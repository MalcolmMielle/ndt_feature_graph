#pragma once

#include <ndt_map/ndt_map.h>
#include <ndt_mcl/3d_ndt_mcl.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
// #include <semrob_rviz/semrob_rviz.h>

// #include <semrob_generic/types.h>
// #include <semrob_generic/utils.h>
// #include <semrob_conversions/conversions.h>
// #include <visualization_msgs/Marker.h>
// #include <tf/transform_datatypes.h>
// #include <Eigen/Core>
// #include <Eigen/LU>
// #include <Eigen/Eigenvalues>
// #include <vector>

#include "motion_model.hpp"


namespace ndt_feature{
inline void assignDefault(visualization_msgs::Marker &m)
{
  m.header.frame_id = "/world";
  m.scale.x = 1;
  m.scale.y = 1;
  m.scale.z = 1;
  m.lifetime = ros::Duration(60.);
}

inline void assignColor(visualization_msgs::Marker &m, int color)
{
  double r,g,b = 0.;
  int color_mod = color % 4;
  
  switch (color_mod)
    {
    case 0:
      r = 1.; g = 0.; b = 0.;
      break;
    case 1:
      r = 0.; g = 1.; b = 0.;
      break;
    case 2:
      r = 0.; g = 0.; b = 1.;
      break;
    case 3:
        r = 1.; g = 0.; b = 1.;
        break;
    default:
      r = 0.5; g = 0.5; b = 0.5;
      break;
    };

  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 0.3;

  if (color < 0) {
    m.color.r = 1.;
    m.color.g = 1.;
    m.color.b = 1.;
    m.color.a = 1.;
  }
}


inline  void drawPose2d(double x, double y, double th, int id, int color, double scale, const std::string &name, ros::Publisher &pub)
{
   visualization_msgs::Marker m;
   assignDefault(m);
   assignColor(m, color);
   
   m.type = visualization_msgs::Marker::ARROW;//CUBE;
   m.action = visualization_msgs::Marker::ADD;
   m.ns = name;
   m.id = id;
   m.pose.position.x = x;
   m.pose.position.y = y;
   m.pose.position.z = 0;
   m.pose.orientation = tf::createQuaternionMsgFromYaw(th);
   m.scale.x = 1.0 * scale;
   m.scale.y = 0.04 * scale;
   m.scale.z = 0.04 * scale;
   pub.publish(m);
}



inline void drawSphere(double x, double y, double radius, int id, const std::string &name, ros::Publisher &pub)
{
   visualization_msgs::Marker m;
   assignDefault(m);
   assignColor(m, 2);

   m.type = visualization_msgs::Marker::SPHERE;
   m.action = visualization_msgs::Marker::ADD;
   m.ns = name;
   m.id = id;
   m.pose.position.x = x;
   m.pose.position.y = y;
   m.pose.position.z = 0;
   m.scale.x = radius*2;
   m.scale.y = radius*2;
   m.scale.z = radius*2;

   pub.publish(m);
}

inline void drawPose2d(const ndt_feature::Pose2d &pose, int id, ros::Publisher &pub)
{
  std::string name = "pose2d";
  drawPose2d(pose(0), pose(1), pose(2), id, id, 0.5, name, pub);
}

inline void drawPose2d(const ndt_feature::Pose2d &pose, int id, const std::string &name, ros::Publisher &pub)
{
  drawPose2d(pose(0), pose(1), pose(2), id, id, 0.5, name, pub);
}

inline void drawPose2d(const ndt_feature::Pose2d &pose, int id, int color, const std::string &name, ros::Publisher &pub)
{
  drawPose2d(pose(0), pose(1), pose(2), id, color, 0.5, name, pub);
}

inline void drawPose2d(const ndt_feature::Pose2d &pose, int id, int color, double scale, const std::string &name, ros::Publisher &pub)
{
  drawPose2d(pose(0), pose(1), pose(2), id, color, scale, name, pub);
}

// Code from user skohlbrecher
inline void drawCovariance(const Eigen::Vector2d& mean, const Eigen::Matrix2d& covMatrix, visualization_msgs::Marker &marker)
{
     marker.pose.position.x = mean[0];
     marker.pose.position.y = mean[1];
     
     Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(covMatrix);
     
     const Eigen::Vector2d& eigValues (eig.eigenvalues());
     const Eigen::Matrix2d& eigVectors (eig.eigenvectors());
     
     float angle = (atan2(eigVectors(1, 0), eigVectors(0, 0)));

     marker.type = visualization_msgs::Marker::CYLINDER;
     
    double lengthMajor = sqrt(eigValues[0]);
    double lengthMinor = sqrt(eigValues[1]);
    
    marker.scale.x = lengthMajor;
    marker.scale.y = lengthMinor;
    marker.scale.z = 0.001;
    
    marker.pose.orientation.w = cos(angle*0.5);
    marker.pose.orientation.z = sin(angle*0.5);
}

inline void drawCovariance(const Eigen::Vector3d& mean, const Eigen::Matrix3d& covMatrix, visualization_msgs::Marker &marker)
{
     Eigen::Vector2d tmp_mean = mean.head(2);
     Eigen::Matrix2d tmp_cov = covMatrix.block(0,0,2,2);
     drawCovariance(tmp_mean, tmp_cov, marker);
}

inline void drawCovariance(const ndt_feature::Pose2dCov &poseCov, visualization_msgs::Marker &marker)
{
     drawCovariance(poseCov.mean, poseCov.cov, marker);
}

inline void drawPose2dCov(const ndt_feature::Pose2dCov &pose, int id, int color, double sigma, const std::string &name, ros::Publisher &pub) 
{
  drawPose2d(pose.mean, id, color, 0.5, "posecov.mean" ,pub);
  // Draw the covariance as a point list.
  visualization_msgs::Marker m;
  assignDefault(m);
  assignColor(m, color);
  m.ns = "posecov.cov";
  m.id = id;
  drawCovariance(pose, m);
  pub.publish(m);
}

// inline void drawPose2d(const ndt_feature::Pose2dContainerInterface &poses, int idx, int id, int color, double scale, const std::string &name, ros::Publisher &pub) {
//   if (idx < 0)
//     return;
//   if (idx >= poses.sizePose2d())
//     return;
//   drawPose2d(poses.getPose2d(idx), id, color, scale, name, pub);
// }
// 
// inline void drawPose2dContainer(const ndt_feature::Pose2dContainerInterface &poses, const std::string &name, int id, ros::Publisher &pub)
// {
//    visualization_msgs::Marker m;
//    assignDefault(m);
//    assignColor(m, id);
//    
//    m.type = visualization_msgs::Marker::ARROW;
//    m.action = visualization_msgs::Marker::ADD;
//    m.ns = name;
//    m.pose.position.z = 0;
//    m.scale.x = 0.5;
//    m.scale.y = 0.02;
//    m.scale.z = 0.02;
// 
// 
//   
//   for (unsigned int i = 0; i < poses.sizePose2d(); i++)
//     {
//       m.id = i;
//       const ndt_feature::Pose2d &p = poses.getPose2d(i);
//       m.pose.position.x = p(0);
//       m.pose.position.y = p(1);
//       m.pose.orientation = tf::createQuaternionMsgFromYaw(p(2));
//       pub.publish(m);
//       usleep(10);
//     }
// }

inline  void drawPosition(double x, double y, double z, int id, ros::Publisher &pub)
 {
   visualization_msgs::Marker m;
   assignDefault(m);
   assignColor(m, id);
   
   m.type = visualization_msgs::Marker::CUBE;
   m.action = visualization_msgs::Marker::ADD;
   m.ns = "position";
   m.id = id;
   m.scale.x = 0.1;
   m.scale.y = 0.1;
   m.scale.z = 0.1;
   m.pose.position.x = x;
   m.pose.position.y = y;
   m.pose.position.z = z;
   m.pose.orientation = tf::createQuaternionMsgFromYaw(0);
   pub.publish(m);

 }

// inline void drawPoint2dContainerAsConnectedLineIncZ_(const semrob_generic::Point2dContainerInterface &points, const std::string &name, int id, int color, double startZ, double incZ, bool addLastPointFirst, ros::Publisher &pub) {
//    
//   if (points.sizePoint2d() == 0)
//      return;
//   visualization_msgs::Marker m;
//   assignDefault(m);
//   assignColor(m, color);
//   m.scale.x = 0.03; // The width of the strip
//   m.type = visualization_msgs::Marker::LINE_STRIP;
//   m.action = visualization_msgs::Marker::ADD;
//   m.ns = name + semrob_generic::toString(id);
//   m.id = id;
// 
//   geometry_msgs::Point p;
//   p.z = startZ;
//   for (unsigned int i = 0; i < points.sizePoint2d(); i++)
//     {
//       p.x = points.getPoint2d(i)(0);
//       p.y = points.getPoint2d(i)(1);
//       p.z += incZ;
//       m.points.push_back(p);
//     }
//   if (addLastPointFirst)
//     m.points.push_back(m.points[0]); // Need to add the first one to 'close the loop'.
//   pub.publish(m);
// }

// inline  void drawPoint2dContainerAsConnectedLineIncZ(const semrob_generic::Point2dContainerInterface &points, const std::string &name, int id, int color, double startZ, double incZ, ros::Publisher &pub) {
//    drawPoint2dContainerAsConnectedLineIncZ_(points, name, id, color, startZ, incZ, true, pub);
//  }

// inline  void drawPoint2dContainerAsConnectedLine(const semrob_generic::Point2dContainerInterface &points, const std::string &name, int id, int color, ros::Publisher &pub)
// {
//   drawPoint2dContainerAsConnectedLineIncZ(points, name, id, color, 0., 0., pub);
// }
// 
// inline  void drawPathInterfaceIncZ(const semrob_generic::PathInterface &path, const std::string &name, int id, int color, double incZ, ros::Publisher &pub)
//  {
//    semrob_generic::Point2dVec pvec = semrob_generic::createPoint2dVecFromPose2dContainerInterface(path);
//    drawPoint2dContainerAsConnectedLineIncZ(pvec, name, id, color, 0., incZ, pub);
// }



inline Eigen::Vector2d getIntersection(double A11, double A12, double A21, double A22, double b1, double b2)
{
  Eigen::Matrix2d A;
  A << A11, A12, A21, A22;
  Eigen::Vector2d b(b1, b2);
  return A.inverse()*b;
}

inline void calcMatrixFormForLine(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, double &A0, double &A1, double &b)
{
    Eigen::Vector2d normal;

    normal(0) = p1(1) - p2(1);
    normal(1) = p2(0) - p1(0);
    normal.normalize();
    b = p2.dot(normal);
    A0 = normal(0);
    A1 = normal(1);
}

inline std::vector<Eigen::Vector2d> getIntersectionsPose2d(const std::vector<double> &A0, const std::vector<double> &A1, const std::vector<double> &b, const ndt_feature::Pose2d& start, const ndt_feature::Pose2d& end, double bOffset)
{
  std::vector<Eigen::Vector2d> intersections;
  double A0_, A1_, b_;
  for (unsigned int i = 0; i < A0.size(); i++) {
    
    calcMatrixFormForLine(ndt_feature::getPosition(start), ndt_feature::getBaseOffset(start, 1.), A0_, A1_, b_);
    intersections.push_back(getIntersection(A0[i], A1[i], A0_, A1_, b[i]+bOffset, b_));
    
    calcMatrixFormForLine(ndt_feature::getPosition(end), ndt_feature::getBaseOffset(end, 1.), A0_, A1_, b_);
    intersections.push_back(getIntersection(A0[i], A1[i], A0_, A1_, b[i]+bOffset, b_));
  }
 return intersections;
}


// inline void drawPolygonConstraint(const semrob_msgs::PolygonConstraint &msg, visualization_msgs::Marker m, ros::Publisher &pub, double bOffset)
// {
//   std::vector<double> A0 = msg.A0;
//   std::vector<double> A1 = msg.A1;
//   std::vector<double> b  = msg.b;
// 
//   if (bOffset != 0)
//     {
//       for (size_t i = 0; i < b.size(); i++)
// 	{
// 	  b[i] += bOffset;
// 	}
//     }
// 
//   assert(A0.size() == A1.size() && A0.size() == b.size());
//   assert(A0.size() > 1); // TODO - add support to do only one constraint
//   // Find intersecting points - here we assume that the lines comes in order.
//   for (size_t i = 0; i < A0.size(); i++)
//     {
//       Eigen::Vector2d inter;
//       if (i == A0.size() -1)
// 	inter = getIntersection(A0[i], A1[i], A0[0], A1[0], b[i], b[0]);
//       else
// 	inter = getIntersection(A0[i], A1[i], A0[i+1], A1[i+1], b[i], b[i+1]);
//       geometry_msgs::Point p;
//       p.x = inter[0];
//       p.y = inter[1];
//       p.z = 0;
//       m.points.push_back(p);
//     }
//   m.points.push_back(m.points[0]); // Need to add the first one to 'close the loop'.
//   pub.publish(m);
// }

// inline void drawConstraintsMarker(const semrob_msgs::RobotConstraints &msg, visualization_msgs::Marker m, visualization_msgs::Marker m2, ros::Publisher &pub, int stepSize)
// {
//   m.type = visualization_msgs::Marker::LINE_STRIP;
//   m.action = visualization_msgs::Marker::ADD;
//   m.ns = "constraints_" + semrob_generic::toString(msg.robot_id) + semrob_generic::toString(msg.goal_id);
//   m2.type = visualization_msgs::Marker::LINE_STRIP;
//   m2.action = visualization_msgs::Marker::ADD;
//   m2.ns = "constraints2_" + semrob_generic::toString(msg.robot_id) + semrob_generic::toString(msg.goal_id);
//     
//   int id = 0;
//   for (size_t i = 0; i < msg.constraints.size(); i+=stepSize)
//     {
//       m.id = id;
//       assignColor(m, i);
//       drawPolygonConstraint(msg.constraints[i], m, pub, 0.);
//       m2.id = id;
//       drawPolygonConstraint(msg.constraints[i], m2, pub, 0.05);
//       id++;
//     }
// }
// 
// inline void drawConstraints(const semrob_msgs::RobotConstraints &msg, ros::Publisher &pub)
// {
//   visualization_msgs::Marker m,m2;
//   assignDefault(m);
//   assignColor(m, msg.robot_id);
//   m.scale.x = 0.02; // The width of the strip
//   m2 = m; // m is right on the constraints where as m2 illustrate an invalid constraint (on the wrong side)
//   m2.scale.x *= 0.3;
//   m2.color.a *= 0.3;
//   drawConstraintsMarker(msg, m, m2, pub, 1);
// } 


inline void drawPointsVec(const std::vector<Eigen::Vector3d> &points, const std::string &name, int id, ros::Publisher &pub)
{
     visualization_msgs::Marker m;
     assignDefault(m);
     assignColor(m, id);
     m.id = id;
     m.ns = name;
     m.type = visualization_msgs::Marker::CUBE_LIST;//POINTS;
     m.action = visualization_msgs::Marker::ADD;

     m.scale.x = 0.2;
     m.scale.y = 0.2;
     m.scale.z = 0.2;
           
     for (size_t i = 0; i < points.size(); i++)
     {
       geometry_msgs::Point p;
       p.x = points[i](0);
       p.y = points[i](1);
       p.z = points[i](2);
       m.points.push_back(p);
     }
     pub.publish(m);
}

inline void drawSteeringWheel(const ndt_feature::Pose2d &pose, const std::string &name, int id, ros::Publisher &pub)
 {
   visualization_msgs::Marker m;
   assignDefault(m);
   assignColor(m, 2);
   
   m.type = visualization_msgs::Marker::ARROW;
   m.action = visualization_msgs::Marker::ADD;
   m.ns = name + "_sw";
   m.id = id;
   m.pose.position.x = pose(0);
   m.pose.position.y = pose(1);
   m.pose.position.z = 0;
   m.pose.orientation = tf::createQuaternionMsgFromYaw(pose(2));
   m.scale.x = 0.1;
   m.scale.y = 0.01;
   m.scale.z = 0.03;
   
   pub.publish(m);
 } 


// inline void drawPathInterface(const semrob_generic::PathInterface &path, const std::string &name, int id, double L, ros::Publisher &pub)
//  {
//    // Draw the pose
//    drawPose2dContainer(path, name, id, pub);
//    // Draw the steering angle as an other arrow - need to get the steering angle offset.
//    for (unsigned int i = 0; i < path.sizePath(); i++)
//      {
//        ndt_feature::Pose2d origin = path.getPose2d(i);
//        ndt_feature::Pose2d offset;
//        offset << L, 0, path.getSteeringAngle(i);
//        ndt_feature::Pose2d sw = semrob_generic::addPose2d(origin, offset);
//        drawSteeringWheel(sw, name, i, pub);
//      }
// }
// 
// inline void drawPositionContainer(const semrob_generic::PositionContainerInterface &points, double scale, const std::string &name, int id, ros::Publisher &pub)
//  {
//     visualization_msgs::Marker m;
//      assignDefault(m);
//      assignColor(m, id);
//      m.id = id;
//      m.ns = name;
//      m.type = visualization_msgs::Marker::CUBE_LIST;//POINTS;
//      m.action = visualization_msgs::Marker::ADD;
// 
//      m.scale.x = scale;
//      m.scale.y = scale;
//      m.scale.z = scale;
//     
//      for (size_t i = 0; i < points.sizePos(); i++)
//      {
//        geometry_msgs::Point p;
//        p.x = points.getPos(i)(0);
//        p.y = points.getPos(i)(1);
//        p.z = points.getPos(i)(2);
//        m.points.push_back(p);
//      }
//      pub.publish(m);
// }
// 
// inline void drawPositionContainer(const semrob_generic::PositionContainerInterface &points, const std::string &name, int id, ros::Publisher &pub)
//  {
//    drawPositionContainer(points, 0.2, name, id, pub);
//  }
// 
// inline void drawPositionContainerAsLineList(const semrob_generic::PositionContainerInterface &points, double scale, const std::string &name, int id, ros::Publisher &pub)
//  {
//     visualization_msgs::Marker m;
//      assignDefault(m);
//      assignColor(m, id);
//      m.id = id;
//      m.ns = name;
//      m.type = visualization_msgs::Marker::LINE_LIST;
//      m.action = visualization_msgs::Marker::ADD;
// 
//      m.scale.x = scale;
//      m.scale.y = scale;
//      m.scale.z = scale;
//     
//      for (size_t i = 0; i < points.sizePos(); i++)
//      {
//        geometry_msgs::Point p;
//        p.x = points.getPos(i)(0);
//        p.y = points.getPos(i)(1);
//        p.z = points.getPos(i)(2);
//        m.points.push_back(p);
//      }
//      pub.publish(m);
// }
// 
// 
// inline void drawTrajectoryWithControlTypeSpeed(const semrob_generic::TrajectoryInterface &traj, int color, int id, int type, double scale, const std::string &name, bool useSteeringVel, ros::Publisher &pub) {
//   visualization_msgs::Marker m;
//   assignDefault(m);
//   assignColor(m, color);
//   m.scale.x = scale; // The width
//   m.type = type;
//   m.action = visualization_msgs::Marker::ADD;
//   m.ns = name;
//   m.id = id;
//   for (size_t i = 0; i < traj.sizeTrajectory(); i++) {
//     geometry_msgs::Point p;
//     p.x = traj.getPose2d(i)[0];
//     p.y = traj.getPose2d(i)[1];
//     if (useSteeringVel) 
//       p.z = traj.getSteeringVel(i);
//     else
//       p.z = traj.getDriveVel(i);
//     m.points.push_back(p);
//   }
//   pub.publish(m);
//  }
// 
// inline void drawSteeringAngleVel(const semrob_generic::TrajectoryInterface &traj, int color, const std::string &name, ros::Publisher &pub) {
//    drawTrajectoryWithControlTypeSpeed(traj, color, 1, 4, 0.02, name, true, pub);
//  }
// 
// inline void drawTrajectoryWithControlType(const semrob_generic::TrajectoryInterface &traj, int color, int id, int type, double scale, const std::string &name, ros::Publisher &pub) {
//    // Draw the trajectory as a line which follows the path using [x,y] and the speed is the z value.
//    drawTrajectoryWithControlTypeSpeed(traj, color, id, type, scale, name, false, pub);
//  }
// 
// inline void drawTrajectoryWithControl(const semrob_generic::TrajectoryInterface &traj, int color, int id, const std::string &name, ros::Publisher &pub) {
//    // Draw the trajectory as a line which follows the path using [x,y] and the speed is the z value.
//    drawTrajectoryWithControlType(traj, color, id, 4, 0.03, name, pub);
//  }
// 
// inline void drawTrajectoryChunksWithControl(const semrob_generic::TrajectoryChunksInterface &chunks, int color, const std::string &name, ros::Publisher &pub) {
//    for (size_t i = 0; i < chunks.sizeChunks(); i++) {
//      drawTrajectoryWithControl(chunks.getChunk(i), color, i, name, pub);
//    }
//    
//    // Draw cubes at the first chunk path points.
//    semrob_generic::PositionVec pts;
//    for (size_t i = 0; i < chunks.sizeChunks(); i++) {
//      pts.push_back(Eigen::Vector3d(chunks.getChunk(i).getPose2d(0)[0],
// 				   chunks.getChunk(i).getPose2d(0)[1],
// 				   chunks.getChunk(i).getDriveVel(0)));
//    }
//    
//    drawPositionContainer(pts, 0.05, name + std::string("_idx0"), 0, pub);
//  }
// 
//  // Draw the trajectory chunk as a position 2d - time step.
// inline void drawTrajectoryChunksUsingTime(const semrob_generic::TrajectoryChunksInterface &chunks, int color, const std::string &name, double timeOffset, double timeToMeterFactor, double dt, ros::Publisher &pub) {
//   
//   semrob_generic::PositionVec pts;
//   int idx = 0;
//   for (size_t i = 0; i < chunks.sizeChunks(); i++) {
//     for (unsigned int j = 0; j < chunks.getChunk(i).sizeTrajectory(); j++) {
//       pts.push_back(Eigen::Vector3d(chunks.getChunk(i).getPose2d(j)[0],
//                                     chunks.getChunk(i).getPose2d(j)[1],
//                                     (timeOffset + idx*dt)*timeToMeterFactor));
//       idx++;
//     }
//   }
//   drawPositionContainer(pts, 0.05, name, 0, pub);
// }

inline void drawPalletWithTime(const ndt_feature::Pose2d &pose, const std::string &frame, int id, const ros::Time &time, ros::Publisher &pub) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = time;
    marker.ns = "pallet";
    marker.id = id;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 2;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.0254;
    marker.scale.y = 0.0254;
    marker.scale.z = 0.0254;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.mesh_resource = "package://euro_pallet/meshes/EuroPallet.dae";

    pub.publish( marker );
 }

inline void drawPallet(const ndt_feature::Pose2d &pose, const std::string &frame, int id, ros::Publisher &pub) {
  drawPalletWithTime(pose, frame, id, ros::Time::now(), pub);
}

inline void drawText(const ndt_feature::Pose2d &pose, const std::string &text, const std::string &name, int id, int color, double height, double size, ros::Publisher &pub) {
   visualization_msgs::Marker m;
   assignDefault(m);
   assignColor(m, color);
   m.pose.position.x = pose(0);
   m.pose.position.y = pose(1);
   m.pose.position.z = height;
   m.pose.orientation = tf::createQuaternionMsgFromYaw(pose(2));
   
   m.ns = name;
   m.id = id;
   m.scale.z = size;
   m.text = text;
   m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
     
   pub.publish(m);

}

// inline void drawPose2dTimes(const ndt_feature::Pose2dContainerInterface &poses, const std::vector<double> &times, double startTime, const std::string &name, int color, double timeScale, double sizeScale, ros::Publisher &pub) {
//   semrob_generic::CoordinatedTimes ct(times);
//   ct.addOffset(-startTime);
//   assert(poses.sizePose2d() == times.size());
//   semrob_generic::PositionVec pts;
//   for (size_t i = 0; i < times.size(); i++) {
//     if (times[i] >= 0.) {
//       pts.push_back(Eigen::Vector3d(poses.getPose2d(i)[0],
// 				    poses.getPose2d(i)[1],
// 				    (times[i] - startTime)*timeScale));
//     }
//   }
//   drawPositionContainer(pts, sizeScale, name, color, pub);
// }
// 
// inline void drawPose2dTimesAsLines(const ndt_feature::Pose2dContainerInterface &poses, const std::vector<double> &times, double startTime, const std::string &name, int color, double timeScale, double sizeScale, ros::Publisher &pub) {
// semrob_generic::CoordinatedTimes ct(times);
//   ct.addOffset(-startTime);
//   assert(poses.sizePose2d() == times.size());
//   semrob_generic::PositionVec pts;
//   for (size_t i = 0; i < times.size(); i++) {
//     if (times[i] >= 0.) {
//       pts.push_back(Eigen::Vector3d(poses.getPose2d(i)[0],
// 				    poses.getPose2d(i)[1],
// 				    0.));
//       pts.push_back(Eigen::Vector3d(poses.getPose2d(i)[0],
// 				    poses.getPose2d(i)[1],
// 				    (times[i] - startTime)*timeScale));
//     }
//   }
//   drawPositionContainerAsLineList(pts, sizeScale, name, color, pub);
// }

// inline void drawPose2dTimes(const ndt_feature::Pose2dContainerInterface &poses, const std::vector<double> &times, double startTime, const std::string &name, int color, double timeScale, ros::Publisher &pub) {
//   drawPose2dTimes(poses, times, startTime, name, color, timeScale, 0.05, pub);
// }
// 
// 
// inline void drawCoordinatedTimes(const ndt_feature::Pose2dContainerInterface &poses, const semrob_generic::CoordinatedTimes &coordTimes, double startTime, const std::string &name, int color, double timeScale, double sizeScale, ros::Publisher &pub) {
//   drawPose2dTimes(poses, coordTimes, startTime, name, color, timeScale, sizeScale, pub);
// }
// 
// inline void drawCoordinatedTimes(const ndt_feature::Pose2dContainerInterface &poses, const semrob_generic::CoordinatedTimes &coordTimes, double startTime, const std::string &name, int color, double timeScale, ros::Publisher &pub) {
//   drawPose2dTimes(poses, coordTimes, startTime, name, color, timeScale, pub);
// }
// 
// inline void drawDeltaTVec(const ndt_feature::Pose2dContainerInterface &poses, const semrob_msgs::DeltaTVec &msg, ros::Publisher &pub) {
//   for (size_t i = 0; i < msg.dts.size(); i++) {
//     std::vector<double> dts = semrob_conversions::getDoubleVecFromDeltaTMsg(msg.dts[i]);
//     semrob_generic::CoordinatedTimes ct = semrob_generic::computeCoordinatedTimesFromDeltaTs(dts);
//     drawCoordinatedTimes(poses, ct, 0., "dts", i, 0.1, pub);
//     usleep(10);
//   }
// }
// 
// 
// inline void drawTrajectoryLinePairs(const semrob_generic::TrajectoryInterface &traj1, const semrob_generic::TrajectoryInterface &traj2, int color, int id, const std::string &name, ros::Publisher &pub) {
//    // Draw a line between the velocity points
//    visualization_msgs::Marker m;
//    assignDefault(m);
//    assignColor(m, 1);
//    m.color.a = 1;
//    m.scale.x = 0.001;
//    m.type = visualization_msgs::Marker::LINE_LIST;
//    m.action = visualization_msgs::Marker::ADD;
//    m.ns = name;
//    m.id = id;
//    assert(traj1.sizeTrajectory() == traj2.sizeTrajectory());
//    for (size_t i = 0; i < traj1.sizeTrajectory(); i++) {
//      geometry_msgs::Point p;
//      p.x = traj1.getPose2d(i)[0];
//      p.y = traj1.getPose2d(i)[1];
//      p.z = traj1.getDriveVel(i);
//      m.points.push_back(p);
// 
//      p.x = traj2.getPose2d(i)[0];
//      p.y = traj2.getPose2d(i)[1];
//      p.z = traj2.getDriveVel(i);
//      m.points.push_back(p);
//    }
//    pub.publish(m);
//    
//  }
// 
// inline void drawTrajectoryRefAndExecPair(const semrob_generic::TrajectoryInterface &ref, const semrob_generic::Trajectory &exec, const std::string &name, ros::Publisher &pub) {
//    
//    // Blue line between the pairs.
//    drawTrajectoryLinePairs(ref, exec, 2, 0, name + std::string("_pairs"), pub);
//    // Green points - ref
//    drawTrajectoryWithControlType(ref, 1, 0, 8, 0.005, name + std::string("_ref"), pub);
//    // Exec points - red
//    drawTrajectoryWithControlType(exec, 0, 0, 8, 0.005, name + std::string("_exec"), pub);
//  }



#if 0
void drawTrajectoryMarker(const semrob_msgs::Trajectory &msg, visualization_msgs::Marker m, ros::Publisher &pub, int stepSize)
{
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::ADD;
  m.ns = "trajectory" + sauna_generic::toString(msg.robot_id) + sauna_generic::toString(msg.goal_id);
  int id = 0;
  for (size_t i = 0; i < msg.trajectory.size(); i+=stepSize)
    {
      m.pose = msg.trajectory[i].pose.pose;
      m.id = id;
      pub.publish(m);
      id++;
    }
}

void drawTrajectory(const sauna_msgs::Trajectory &msg, ros::Publisher &pub)
{
  visualization_msgs::Marker m;
  assignDefault(m);
  assignColor(m, msg.robot_id);
  drawTrajectoryMarker(msg, m, pub, 10);
}

void drawTrajectories(const sauna_msgs::Trajectories &msg, ros::Publisher &pub)
{
  for (size_t i = 0; i < msg.trajectories.size(); i++)
    {
      drawTrajectory(msg.trajectories[i], pub);
    }
}

void drawPath(const sauna_msgs::Path &msg, ros::Publisher &pub)
{
  visualization_msgs::Marker m;
  assignDefault(m);
  assignColor(m, msg.robot_id);

  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::ADD;
  ros::Duration d(10.);
  m.ns = "path" + sauna_generic::toString(msg.robot_id) + sauna_generic::toString(msg.goal_id);
  int id = 0;
  for (size_t i = 0; i < msg.path.size(); i++)
    {
      m.pose = msg.path[i].pose;
      m.id = id;
      pub.publish(m);
      id++;
    }
}

void drawPath2(const sauna_msgs::Path &msg, ros::Publisher &pub)
{
  visualization_msgs::Marker m;
  assignDefault(m);
  assignColor(m, 4);

  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::ADD;
  m.ns = "path2_" + sauna_generic::toString(msg.robot_id) + sauna_generic::toString(msg.goal_id);
  int id = 0;
  for (size_t i = 0; i < msg.path.size(); i++)
    {
      m.pose = msg.path[i].pose;
      m.id = id;
      pub.publish(m);
      id++;
    }
}
 / Draw a constraints which do not have to intersect (as in drawPolygonConstraint), we here need the start and and pose from which the constraints will be drawn.
void drawOpenConstraint(const std::vector<double> &A0, const std::vector<double> &A1, const std::vector<double> &b, const sauna_generic::Pose2d& start, const sauna_generic::Pose2d& end, visualization_msgs::Marker m, ros::Publisher &pub, double bOffset)
{
  std::vector<Eigen::Vector2d> inter = getIntersectionsPose2d(A0, A1, b, start, end, bOffset);
  for (unsigned int i = 0; i < inter.size(); i++) {
    geometry_msgs::Point p;
    p.x = inter[i](0);
    p.y = inter[i](1);
    p.z = 0;
    m.points.push_back(p);
  }
  pub.publish(m);
}

void drawControllerTrajectoryChunk(const sauna_msgs::ControllerTrajectoryChunk &msg, ros::Publisher &pub)
{
  visualization_msgs::Marker m,m2;
  assignDefault(m);
  assignColor(m, msg.robot_id);
  m.scale.x = 0.02; // The width of the strip

  m.type = visualization_msgs::Marker::LINE_LIST;
  m.action = visualization_msgs::Marker::ADD;
  m2 = m; // m is right on the constraints where as m2 illustrate an invalid constraint (on the wrong side)
  m.ns = "oconstraints_" + sauna_generic::toString(msg.robot_id) + sauna_generic::toString(msg.traj_id) + " " + sauna_generic::toString(msg.sequence_num);
  m2.ns = "oconstraints2_" + sauna_generic::toString(msg.robot_id) + sauna_generic::toString(msg.traj_id) + " " + sauna_generic::toString(msg.sequence_num);
  m2.scale.x *= 0.3;
  m2.color.a *= 0.3;
  
  sauna_generic::Pose2d start = sauna_conversions::createPose2dFromControllerStateMsg(msg.steps[0].state);
  sauna_generic::Pose2d stop  = sauna_conversions::createPose2dFromControllerStateMsg(msg.steps[msg.steps.size()-1].state);
  
  drawOpenConstraint(msg.constraints.spatial_coef_a0,
		     msg.constraints.spatial_coef_a1,
		     msg.constraints.spatial_coef_b,
		     start,
		     stop, m, pub, 0.);

  drawOpenConstraint(msg.constraints.spatial_coef_a0,
		     msg.constraints.spatial_coef_a1,
		     msg.constraints.spatial_coef_b,
		     start, stop, m2, pub, 0.05);
}






 void drawTrajectoryLinePairs(const sauna_generic::TrajectoryInterface &traj1, const sauna_generic::TrajectoryInterface &traj2, int color, int id, const std::string &name, ros::Publisher &pub) {
   // Draw a line between the velocity points
   visualization_msgs::Marker m;
   assignDefault(m);
   assignColor(m, 1);
   m.color.a = 1;
   m.scale.x = 0.001;
   m.type = visualization_msgs::Marker::LINE_LIST;
   m.action = visualization_msgs::Marker::ADD;
   m.ns = name;
   m.id = id;
   assert(traj1.sizeTrajectory() == traj2.sizeTrajectory());
   for (size_t i = 0; i < traj1.sizeTrajectory(); i++) {
     geometry_msgs::Point p;
     p.x = traj1.getPose2d(i)[0];
     p.y = traj1.getPose2d(i)[1];
     p.z = traj1.getDriveVel(i);
     m.points.push_back(p);

     p.x = traj2.getPose2d(i)[0];
     p.y = traj2.getPose2d(i)[1];
     p.z = traj2.getDriveVel(i);
     m.points.push_back(p);
   }
   pub.publish(m);
   
 }

 void drawTrajectoryRefAndExecPair(const sauna_generic::TrajectoryInterface &ref, const sauna_generic::Trajectory &exec, const std::string &name, ros::Publisher &pub) {
   
   // Blue line between the pairs.
   drawTrajectoryLinePairs(ref, exec, 2, 0, name + std::string("_pairs"), pub);
   // Green points - ref
   drawTrajectoryWithControlType(ref, 1, 0, 8, 0.005, name + std::string("_ref"), pub);
   // Exec points - red
   drawTrajectoryWithControlType(exec, 0, 0, 8, 0.005, name + std::string("_exec"), pub);
 }



 void drawLinesBetweenPointPairs(const sauna_generic::Point2dContainerInterface &points, const std::string &name, int color, ros::Publisher &pub) {
   visualization_msgs::Marker m,m2;
   assignDefault(m);
   assignColor(m, color);
   m.scale.x = 0.02; // The width of the strip
   
   m.type = visualization_msgs::Marker::LINE_LIST;
   m.action = visualization_msgs::Marker::ADD;
   m.ns = name;
   
   geometry_msgs::Point p;
   for (unsigned int i = 0; i < points.sizePoint2d(); i++)
     {
       p.x = points.getPoint2d(i)(0);
       p.y = points.getPoint2d(i)(1);
       p.z = 0.;
       m.points.push_back(p);
     }
   pub.publish(m);
   
 }



void drawText2(const sauna_generic::Pose2d &pose, const std::string &text, const std::string &name, int id, int color, double height, double size, ros::Publisher &pub) {
   visualization_msgs::Marker m;
   assignDefault(m);
   assignColor(m, color);
   m.pose.position.x = pose(0);
   m.pose.position.y = pose(1);
   m.pose.position.z = height;
   m.pose.orientation = tf::createQuaternionMsgFromYaw(pose(2));
   
   m.ns = name;// + sauna_generic::toString(id);
   m.id = id;
   m.scale.z = size;
   m.text = text;
   m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
     
   pub.publish(m);

}

 void drawText2(const sauna_generic::Pose2d &pose, const std::string &text, const std::string &name, int id, double height, double size, ros::Publisher &pub) 
 {
   drawText2(pose, text, name, id, id, height, size, pub);
 }


 void drawText(const sauna_generic::Pose2d &pose, const std::string &text, int id, double height, double size, ros::Publisher &pub) 
 {
   std::string name("text");
   drawText2(pose, text, name, id, height, size, pub);
 }

 void drawTarget(const sauna_msgs::RobotTarget &msg, ros::Publisher &pub)
 {
   visualization_msgs::Marker m;
   assignDefault(m);
   assignColor(m, msg.goal_id);

   m.type = visualization_msgs::Marker::ARROW;//CUBE;
   m.action = visualization_msgs::Marker::ADD;
   m.ns = "target_" + sauna_generic::toString(msg.robot_id);
   m.id = msg.goal_id;
   m.pose = msg.goal.pose;
   m.scale.x *= 2;
   m.scale.y *= 2;
   m.scale.z *= 2;
     
   pub.publish(m);

   /* // Compute the times */
   /* ros::Time current_time = ros::Time::now(); */
   /* double es_time = (msg.start_earliest - current_time).toSec(); */
   /* double ds_time = (msg.start_deadline - current_time).toSec(); */
   /* double eg_time = (msg.goal_earliest - current_time).toSec(); */
   /* double dg_time = (msg.goal_deadline - current_time).toSec(); */

   /* // START    */
   /* std::string text; */
   /* text = "S:[" + sauna_generic::toString(es_time) + "|" + sauna_generic::toString(ds_time) + "](RID:" + sauna_generic::toString(msg.robot_id) + ",GID:" + sauna_generic::toString(msg.goal_id) + ")"; */
   /* drawText2(sauna_conversions::createPose2dFromMsg(msg.start.pose), */
   /* 	     text, std::string("target_es"), msg.robot_id, 0.6, 0.3, pub); */

   /* text = "G:[" + sauna_generic::toString(eg_time) + "|" + sauna_generic::toString(dg_time) + "](RID:" + sauna_generic::toString(msg.robot_id) + ",GID:" + sauna_generic::toString(msg.goal_id) + ")"; */
   /* drawText2(sauna_conversions::createPose2dFromMsg(msg.goal.pose), */
   /* 	     text, std::string("target_eg"), msg.robot_id, 0.6, 0.3, pub); */

   std::string text;
   text = "S:(RID:" + sauna_generic::toString(msg.robot_id) + ",GID:" + sauna_generic::toString(msg.goal_id) + ")";
   drawText2(sauna_conversions::createPose2dFromMsg(msg.start.pose),
   	     text, std::string("target_es"), msg.robot_id, 0.6, 0.3, pub);

   text = "G:(RID:" + sauna_generic::toString(msg.robot_id) + ",GID:" + sauna_generic::toString(msg.goal_id) + ")";
   drawText2(sauna_conversions::createPose2dFromMsg(msg.goal.pose),
   	     text, std::string("target_eg"), msg.robot_id, 0.6, 0.3, pub);

 }

void drawCoordinatedTimesAsText(const sauna_generic::Pose2dContainerInterface &poses, const std::vector<double> &coordTimes, const std::string &name, int color, double height, ros::Publisher &pub) {
  assert(poses.sizePose2d() == coordTimes.size());
  int id = 0;
  for (unsigned int i = 0; i < coordTimes.size(); i++) {
    if (coordTimes[i] >= 0.) {
      std::string text = sauna_generic::toString(coordTimes[i]) + "s";
      drawText2(poses.getPose2d(i),
		text, name, id++, color, height, 0.1, pub);
    }
    else {
      std::string text = "-";
      drawText2(poses.getPose2d(i), text, name, id++, color, height, 0.1, pub);
    }
  }
}



#endif
	
	
}





namespace ndt_visualisation {

  // Contains a set of useful functions to generate markers from NDT related classes.

  // Some helper functions to convert to the position etc. utilized in the markers.
  inline geometry_msgs::Point toPointFromTF (const tf::Vector3& p)
  {
    geometry_msgs::Point pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
    return pt;
  }
  
  inline geometry_msgs::Point toPointFromEigen (const Eigen::Vector3d &p) 
  {
    geometry_msgs::Point pt;
    pt.x = p(0);
    pt.y = p(1);
    pt.z = p(2);
    return pt;
  }

  inline void assignDefault(visualization_msgs::Marker &m)
  {
    m.header.frame_id = "/world";
    m.scale.x = 1;
    m.scale.y = 1;
    m.scale.z = 1;
    m.lifetime = ros::Duration(60.);
  }
  
  inline void assignColor(visualization_msgs::Marker &m, int color)
  {
    double r,g,b = 0.;
    int color_mod = color % 3;
    
    switch (color_mod)
      {
      case 0:
      r = 1.; g = 0.; b = 0.;
      break;
      case 1:
	r = 0.; g = 1.; b = 0.;
	break;
      case 2:
	r = 0.; g = 0.; b = 1.;
	break;
	
      default:
	r = 0.5; g = 0.5; b = 0.5;
	break;
      };
    
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 0.3;
    
    if (color < 0) {
      m.color.r = 1.;
      m.color.g = 1.;
      m.color.b = 1.;
      m.color.a = 1.;
    }
  }
  
  /* typedef std::vector<std_msgs::ColorRGBA> ColorVec; */
  /* ColorVec initColors () */
  /* { */
  /*   ColorVec colors(2); */
  /*   colors[0].r = 0.5; */
  /*   colors[0].g = 1.0; */
  /*   colors[0].a = 1.0; */
  /*   colors[1].r = 1.0; */
  /*   colors[1].g = 1.0; */
  /*   colors[1].a = 1.0; */
  /*   return colors; */
  /* } */

  // Visualize the ndt cells as a set of 3 lines drawn along the eigen vectors.
  inline visualization_msgs::Marker markerNDTCells (std::vector<lslgeneric::NDTCell*> cells/*, tf::Pose& pose*/, const visualization_msgs::Marker &marker)
    {
      visualization_msgs::Marker m = marker;
      m.type = visualization_msgs::Marker::LINE_LIST;
      m.scale.x = 0.02;

      for (size_t i = 0; i < cells.size(); i++)	{
	if(!cells[i]->hasGaussian_) continue;

        //	cells[i]->rescaleCovariance(); // needed?

	Eigen::Vector3d mean = cells[i]->getMean();
	Eigen::Matrix3d evecs = cells[i]->getEvecs();
	Eigen::Vector3d evals = cells[i]->getEvals();
	for (size_t j = 0; j < 3; j++) {
	  double scale = evals(j);
	  if (scale < 0.0001)
	    continue;
	  scale = sqrt(scale);
	  Eigen::Vector3d offset = evecs.col(j) * scale;
	  Eigen::Vector3d p1 = mean - offset;
	  Eigen::Vector3d p2 = mean + offset;
	  m.points.push_back(toPointFromEigen(p1));
	  m.points.push_back(toPointFromEigen(p2));

	  //	  tf::Vector3 p1_, p2_;
	  //	  tf::vectorEigenToTF (p1, p1_);
	  //	  tf::vectorEigenToTF (p2, p2_);
	  //	  m.points.push_back(toPointFromTF(pose*p1_));
	  //	  m.points.push_back(toPointFromTF(pose*p2_));
	}
      }
      return m;
      
    }

 // Visualize the ndt cells as a set of 3 lines drawn along the eigen vectors.
inline visualization_msgs::Marker markerNDTCells (std::vector<lslgeneric::NDTCell*> cells, const Eigen::Affine3d &pose, const visualization_msgs::Marker &marker)
    {
      visualization_msgs::Marker m = marker;
      m.type = visualization_msgs::Marker::LINE_LIST;
      m.scale.x = 0.02;

      for (size_t i = 0; i < cells.size(); i++)	{
	if(!cells[i]->hasGaussian_) continue;

	Eigen::Vector3d mean = cells[i]->getMean();
	Eigen::Matrix3d evecs = cells[i]->getEvecs();
	Eigen::Vector3d evals = cells[i]->getEvals();
	for (size_t j = 0; j < 3; j++) {
	  double scale = evals(j);
	  if (scale < 0.0001)
	    continue;
	  scale = sqrt(scale);
	  Eigen::Vector3d offset = evecs.col(j) * scale;
	  Eigen::Vector3d p1 = mean - offset;
	  Eigen::Vector3d p2 = mean + offset;
	  m.points.push_back(toPointFromEigen(pose*p1));
	  m.points.push_back(toPointFromEigen(pose*p2));
	}
      }
      return m;
      
    }

   inline visualization_msgs::Marker markerNDTCells (std::vector<lslgeneric::NDTCell*> cells) 
    {
      visualization_msgs::Marker m;
      assignDefault(m);
      m.ns = "NDTCells";
      return markerNDTCells(cells, m);
    }

inline visualization_msgs::Marker markerNDTCells( lslgeneric::NDTMap &map, int id, const std::string &name) {
    visualization_msgs::Marker m;
    assignDefault(m);
    assignColor(m, id);
    m.id = id;
    m.ns = name;
    std::vector<lslgeneric::NDTCell*> cells = map.getAllCells();
    visualization_msgs::Marker ret = markerNDTCells(cells, m);
    for (std::vector<lslgeneric::NDTCell*>::const_iterator it = cells.begin(); it != cells.end(); it++) {
      delete *it;
    }
    return  ret;
}

inline visualization_msgs::Marker markerNDTCells( lslgeneric::NDTMap &map, const Eigen::Affine3d &pose, int id, const std::string &name) {
    visualization_msgs::Marker m;
    assignDefault(m);
    assignColor(m, id);
    m.id = id;
    m.ns = name;
    std::vector<lslgeneric::NDTCell*> cells = map.getAllCells();
    visualization_msgs::Marker ret = markerNDTCells(cells, pose, m);
    for (std::vector<lslgeneric::NDTCell*>::const_iterator it = cells.begin(); it != cells.end(); it++) {
      delete *it;
    }
    return  ret;
}

  inline visualization_msgs::Marker markerNDTCells (lslgeneric::NDTMap &map, int id) 
  {
    return  markerNDTCells(map, id, std::string("NDTMap"));
  }

                                                  
 // Visualize the ndt cells as a set of 3 lines drawn along the eigen vectors.
inline void markerNDTCells2 (std::vector<lslgeneric::NDTCell*> cells, const Eigen::Affine3d &pose, visualization_msgs::Marker &m)
    {
      m.type = visualization_msgs::Marker::LINE_LIST;
      m.scale.x = 0.02;

      for (size_t i = 0; i < cells.size(); i++)	{
	if(!cells[i]->hasGaussian_) continue;

	Eigen::Vector3d mean = cells[i]->getMean();
	Eigen::Matrix3d evecs = cells[i]->getEvecs();
	Eigen::Vector3d evals = cells[i]->getEvals();
	for (size_t j = 0; j < 3; j++) {
	  double scale = evals(j);
	  if (scale < 0.0001)
	    continue;
	  scale = sqrt(scale);
	  Eigen::Vector3d offset = evecs.col(j) * scale;
	  Eigen::Vector3d p1 = mean - offset;
	  Eigen::Vector3d p2 = mean + offset;
	  m.points.push_back(toPointFromEigen(pose*p1));
	  m.points.push_back(toPointFromEigen(pose*p2));
	}
      }
    }

inline void markerNDTCells2( lslgeneric::NDTMap &map, const Eigen::Affine3d &pose, int id, const std::string &name, visualization_msgs::Marker &m) {
    assignDefault(m);
    assignColor(m, id);
    m.id = id;
    m.ns = name;
    std::vector<lslgeneric::NDTCell*> cells = map.getAllCells();
    markerNDTCells2(cells, pose, m);
    for (std::vector<lslgeneric::NDTCell*>::const_iterator it = cells.begin(); it != cells.end(); it++) {
      delete *it;
    }
}

  //! Draw correspondance lines between the NDTCells
  inline visualization_msgs::Marker markerCellVectorCorrespondances (lslgeneric::NDTMap &map1, lslgeneric::NDTMap &map2, const std::vector<std::pair<int, int> > &corr) 
    {
      visualization_msgs::Marker m;
      assignDefault(m);
      m.ns = "NDTCellCorrs";
      m.id = 1;
      m.type = visualization_msgs::Marker::LINE_LIST;
      m.scale.x = 0.005;
      m.color.r = m.color.a = 1.0;
      m.color.g = 0.2;

      for (size_t i = 0; i < corr.size(); i++) {
	
	lslgeneric::NDTCell* cell1 = map1.getCellIdx(corr[i].first);
	lslgeneric::NDTCell* cell2 = map2.getCellIdx(corr[i].second);
	if (cell1 == NULL || cell2 == NULL) {
	  ROS_WARN("Failed to get cells using NDTMap::getCellIdx()!");
	  continue;
	}
	m.points.push_back(toPointFromEigen(cell1->getMean()));
	m.points.push_back(toPointFromEigen(cell2->getMean()));
      }
      return m;
  }

inline visualization_msgs::Marker markerMeanCovariance2d(const Eigen::Vector3d &mean, const Eigen::Matrix3d &cov, double scale, int id, int color) {
  visualization_msgs::Marker m;
  assignDefault(m);
  m.ns = "mean_cov";
  m.id = id;
  m.color.r = m.color.a = 1.0;
  m.color.g = 0.2;
  if (color >= 0) {
    assignColor(m, color);
  }
  Eigen::MatrixXd cov_scaled = cov*scale;
  
  ndt_feature::drawCovariance(mean, cov_scaled, m);
  return m;
}

inline visualization_msgs::Marker markerParticlesNDTMCL3D(const NDTMCL3D &mcl, int color, const std::string& ns) {
  visualization_msgs::Marker m;
  assignDefault(m);
  m.ns = ns;
  m.id = 0;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.scale.x = 0.005;
  m.color.g = m.color.a = 1.0;
  m.color.r = 0.6; m.color.b = 0.8;
  if (color >= 0) {
    assignColor(m, color);
  }
  for (unsigned int i = 0; i < mcl.pf.size(); i++) {
    const Eigen::Affine3d &T = mcl.pf.pcloud[i].T;
    m.points.push_back(toPointFromEigen(T.translation()));
    // Length of the line -> 0.3.
    Eigen::Vector3d p2 = T*Eigen::Vector3d(0.3, 0., 0.);
    m.points.push_back(toPointFromEigen(p2));
  }  
  return m;
}


} // namespace

