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
#include <tf/transform_datatypes.h>
#include <boost/foreach.hpp>

namespace sm=sensor_msgs;
namespace gm=geometry_msgs;
namespace vm=visualization_msgs;

using std::vector;

typedef boost::shared_ptr<LaserReading> LaserPtr;
typedef vector<double> DoubleVec;
typedef vector<InterestPoint*> InterestPointVec;

namespace flirtlib_ros
{

geometry_msgs::Pose toRos(const OrientedPoint2D &pose) 
{
    geometry_msgs::Pose p;
    p.position.x = pose.x;
    p.position.y = pose.y;
    p.position.z = 0.;
    p.orientation = tf::createQuaternionMsgFromYaw(pose.theta);
    return p;
}


OrientedPoint2D fromRos(const geometry_msgs::Pose &pose)
{
    OrientedPoint2D p;
    p.x = pose.position.x;
    p.y = pose.position.y;
    p.theta = tf::getYaw(pose.orientation);
    return p;
}


LaserPtr fromRos(const sm::LaserScan& m)
{
  const unsigned n = m.ranges.size();
  DoubleVec angles(n);
  DoubleVec ranges(n);
  for (unsigned i=0; i<n; i++)
  {
    angles[i] = m.angle_min + i*m.angle_increment;
    ranges[i] = m.ranges[i];
  }
  LaserPtr reading(new LaserReading(angles, ranges, m.header.stamp.toSec(),
                                    "ros_laser", "ros_robot"));
  return reading;
}



typedef vector<std_msgs::ColorRGBA> ColorVec;
ColorVec initColors ()
{
  ColorVec colors(2);
  colors[0].r = 0.5;
  colors[0].g = 1.0;
  colors[0].a = 1.0;
  colors[1].r = 1.0;
  colors[1].g = 1.0;
  colors[1].a = 1.0;
  return colors;
}



vector<vm::Marker> poseMarkers (const vector<gm::Pose>& poses)
{
  vector<vm::Marker> markers;

  for (unsigned id=0; id<100; id++)
  {
    vm::Marker m;
    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();
    m.ns="flirtlib.poses";
    m.id = id;
    m.type = vm::Marker::ARROW;
    m.color.r = m.color.b = .15;
    m.color.g = .65;
    m.color.a = 1;
    m.scale.x = m.scale.y = 2.0;
    m.scale.z = 1.0;
    if (id < poses.size())
    {
      m.pose = poses[id];
    }
    else
    {
      m.pose.orientation.w = 1.0;
      m.action = vm::Marker::DELETE;
    }

    markers.push_back(m);
  }

  return markers;
}

gm::Point toPoint (const tf::Vector3& p)
{
  gm::Point pt;
  pt.x = p.x();
  pt.y = p.y();
  pt.z = p.z();
  return pt;
}


// Generate visualization markers for the interest points
// id is 0 or 1, and controls color and orientation to distinguish between
// the two scans
vm::Marker interestPointMarkers (const InterestPointVec& pts, const gm::Pose& pose, const unsigned id)
{
  tf::Transform trans;
  tf::poseMsgToTF(pose, trans);
  static ColorVec colors = initColors();
  vm::Marker m;
  m.header.frame_id = "/map";
  m.header.stamp = ros::Time::now();
  m.ns = "flirtlib";
  m.id = id;
  m.type = vm::Marker::LINE_LIST;
  m.scale.x = 0.02;
  m.color = colors[id];
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
      ROS_ASSERT(id==1);
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
      m.points.push_back(toPoint(trans*pt0));
      m.points.push_back(toPoint(trans*pt1));
    }
  }

  return m;
}

inline
gm::Point toPoint (const Point2D& p)
{
  gm::Point pt;
  pt.x = p.x;
  pt.y = p.y;
  return pt;
}

inline
Point2D toPoint2D (const gm::Point& p)
{
  return Point2D(p.x, p.y);
}

vector<Vector> toRos (const vector<vector<double> >& h)
{
  vector<Vector> m;
  m.resize(h.size());
  for (unsigned i=0; i<h.size(); i++)
  {
    const vector<double>& v = h[i];
    Vector& v2 = m[i];
    vector<double>& vec = v2.vec;
    vec = v;
  }
  return m;
}

vector<vector<double> > fromRos (const vector<Vector>& m)
{
  vector<vector<double> > h;
  h.resize(m.size());
  for (unsigned i=0; i<m.size(); i++)
    h[i] = m[i].vec;
  return h;
}

DescriptorRos toRos (const Descriptor* descriptor)
{
  const BetaGrid* desc = dynamic_cast<const BetaGrid*>(descriptor);
  ROS_ASSERT_MSG(desc,"Descriptor was not of type BetaGrid");
  ROS_ASSERT_MSG(dynamic_cast<const SymmetricChi2Distance<double>*>
    (desc->getDistanceFunction()),
                 "Distance function was not of type SymmetricChi2Distance");
  DescriptorRos m;
  m.hist = toRos(desc->getHistogram());
  m.variance = toRos(desc->getVariance());
  m.hit = toRos(desc->getHit());
  m.miss = toRos(desc->getMiss());
  return m;
}

Descriptor* fromRos (const DescriptorRos& m)
{
  BetaGrid* desc = new BetaGrid();
  desc->setDistanceFunction(new SymmetricChi2Distance<double>());
  desc->getHistogram() = fromRos(m.hist);
  desc->getVariance() = fromRos(m.variance);
  desc->getHit() = fromRos(m.hit);
  desc->getMiss() = fromRos(m.miss);
  return desc;
}

InterestPointRos toRos (const InterestPoint& pt)
{
  InterestPointRos m;

  m.pose.x = pt.getPosition().x;
  m.pose.y = pt.getPosition().y;
  m.pose.theta = pt.getPosition().theta;

  m.support_points.reserve(pt.getSupport().size());
  BOOST_FOREACH (const Point2D& p, pt.getSupport()) 
    m.support_points.push_back(toPoint(p));

  m.scale = pt.getScale();
  m.scale_level = pt.getScaleLevel();
  m.descriptor = toRos(pt.getDescriptor());
  
  return m;
}


InterestPoint* fromRos (const InterestPointRos& m)
{
  OrientedPoint2D pose(m.pose.x, m.pose.y, m.pose.theta);
  Descriptor* descriptor = fromRos(m.descriptor);
  InterestPoint* pt = new InterestPoint(pose, m.scale, descriptor);
  pt->setScaleLevel(m.scale_level);
  vector<Point2D> support_points(m.support_points.size());
  transform(m.support_points.begin(), m.support_points.end(),
            support_points.begin(), toPoint2D);
  pt->setSupport(support_points);
  return pt;
}

RefScan fromRos (const RefScanRos& m)
{
  sm::LaserScan::ConstPtr scan(new sm::LaserScan(m.scan));
  InterestPointVec pts;
  BOOST_FOREACH (const InterestPointRos& p, m.pts)
    pts.push_back(fromRos(p));
  return RefScan(scan, m.pose, pts);
}

RefScanRos toRos (const RefScan& ref)
{
  RefScanRos m;

  m.scan = *ref.scan;
  m.pose = ref.pose;
  BOOST_FOREACH (boost::shared_ptr<InterestPoint> p, ref.pts) 
    m.pts.push_back(toRos(*p));
  return m;
}

vector<RefScan> fromRos (const ScanMap& scan_map)
{
  vector<RefScan> scans;
  BOOST_FOREACH (const RefScanRos& scan, scan_map.scans) 
    scans.push_back(fromRos(scan));
  return scans;
}

ScanMap toRos (const vector<RefScan>& scans)
{
  ScanMap m;
  BOOST_FOREACH (const RefScan& scan, scans) 
    m.scans.push_back(toRos(scan));
  return m;
}


RefScan::RefScan (sm::LaserScan::ConstPtr scan, const gm::Pose& pose,
                  vector<InterestPoint*>& points) :
  scan(scan), pose(pose), raw_pts(points)
{
  pts.reserve(points.size());
  BOOST_FOREACH (InterestPoint* p, points)
    pts.push_back(boost::shared_ptr<InterestPoint>(p));
}

} // namespace
