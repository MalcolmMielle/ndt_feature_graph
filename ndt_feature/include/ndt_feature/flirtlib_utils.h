#ifndef NDT_FEATURE_FLIRTLIB_UTILS_H
#define NDT_FEATURE_FLIRTLIB_UTILS_H

#include <flirtlib_ros/flirtlib.h>
#include <flirtlib_ros/conversions.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/foreach.hpp>

typedef std::vector<InterestPoint*> InterestPointVec;
typedef std::pair<InterestPoint*, InterestPoint*> Correspondence;
typedef std::vector<Correspondence> Correspondences;

namespace ndt_feature {

inline SimpleMinMaxPeakFinder* createPeakFinder ()
{
  return new SimpleMinMaxPeakFinder(0.34, 0.001);
}

inline Detector* createDetector (SimpleMinMaxPeakFinder* peak_finder)
{
  const double scale = 5.0;
  const double dmst = 2.0;
  const double base_sigma = 0.2;
  const double sigma_step = 1.4;
  CurvatureDetector* det = new CurvatureDetector(peak_finder, scale, base_sigma,
                                                 sigma_step, dmst);
  det->setUseMaxRange(false);
  return det;
}

inline DescriptorGenerator* createDescriptor (HistogramDistance<double>* dist)
{
  const double min_rho = 0.02;
  const double max_rho = 1.;//0.5;
  const double bin_rho = 4;
  const double bin_phi = 12;
  BetaGridGenerator* gen = new BetaGridGenerator(min_rho, max_rho, bin_rho,
                                                 bin_phi);
  gen->setDistanceFunction(dist);
  return gen;
}


inline   void convertOrientedPoint2DToEigen(const OrientedPoint2D& pt, Eigen::Affine3d &T) {
    // Take a tour using tf.
    tf::Quaternion q = tf::createQuaternionFromYaw(pt.theta);
    Eigen::Quaterniond qd;
    tf::quaternionTFToEigen(q, qd);
    T = Eigen::Translation3d(pt.x, pt.y, 0.) * qd;
  }
  
inline void convertEigenToOrientedPoint2D(const Eigen::Affine3d &T, OrientedPoint2D &pt) {
  pt.x = T.translation()[0];
  pt.y = T.translation()[1];
  pt.theta = T.rotation().eulerAngles(0,1,2)[2];
}

inline void moveInterestPointVec(const Eigen::Affine3d &T, InterestPointVec &pts) {
  // Move the interest points to T.
  BOOST_FOREACH (InterestPoint* p, pts) {
    Eigen::Affine3d pt;
    convertOrientedPoint2DToEigen(p->getPosition(), pt);
    OrientedPoint2D t;
    convertEigenToOrientedPoint2D(T*pt, t);
    p->setPosition(t);
  }
}

inline bool validOrientedPoint2D(const OrientedPoint2D &p) {
  if (std::isnan(p.x))
    return false;
  if (std::isnan(p.y))
    return false;
  if (std::isnan(p.theta))
    return false;

  return true;
}


} // namespace

#endif
