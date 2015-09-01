#ifndef NDT_FEATURE_FLIRTLIB_UTILS_H
#define NDT_FEATURE_FLIRTLIB_UTILS_H

#include <flirtlib_ros/flirtlib.h>
#include <flirtlib_ros/conversions.h>
#include <tf_conversions/tf_eigen.h>


typedef std::vector<InterestPoint*> InterestPointVec;
typedef std::pair<InterestPoint*, InterestPoint*> Correspondence;
typedef std::vector<Correspondence> Correspondences;

namespace ndt_feature {

SimpleMinMaxPeakFinder* createPeakFinder ()
{
  return new SimpleMinMaxPeakFinder(0.34, 0.001);
}

Detector* createDetector (SimpleMinMaxPeakFinder* peak_finder)
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

DescriptorGenerator* createDescriptor (HistogramDistance<double>* dist)
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


  void convertOrientedPoint2DToEigen(const OrientedPoint2D& pt, Eigen::Affine3d &T) {
    // Take a tour using tf.
    tf::Quaternion q = tf::createQuaternionFromYaw(pt.theta);
    Eigen::Quaterniond qd;
    tf::quaternionTFToEigen(q, qd);
    T = Eigen::Translation3d(pt.x, pt.y, 0.) * qd;
  }
  



} // namespace

#endif
