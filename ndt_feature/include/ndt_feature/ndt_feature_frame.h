#ifndef NDT_FEATURE_FRAME_HH
#define NDT_FEATURE_FRAME_HH

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <flirtlib_ros/flirtlib.h>
#include <ndt_feature/flirtlib_utils.h>
#include <ndt_feature/conversions.h>
#include <ndt_map/pointcloud_utils.h>

namespace ndt_feature {

  //! Class that holds a combination of 'normal' NDT occupancy maps with features.
  class NDTFeatureFrame {
  public:
    //! The raw points
    pcl::PointCloud<pcl::PointXYZ> pc;
    //! OCcupancy map
    //    NDTMap<pcl::PointXYZ> ndt_map;
    InterestPointVec pts;
    Eigen::Affine3d odom;
    Eigen::Affine3d gt;
    Eigen::Affine3d pose; // This is what is optimized
    //! Accumulated distance
    double accu_dist;
  };

  inline void distanceBetweenAffine3d(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2, double &dist, double &angularDist) {
    Eigen::Affine3d tmp = (p1.inverse() * p2);
    dist = tmp.translation().norm();
    angularDist = tmp.rotation().eulerAngles(0,1,2).norm();
  }

  //! Matching (sensor coords).
  inline double ndtFeatureFrameMatchingFLIRT(const NDTFeatureFrame &fixed, const NDTFeatureFrame &moving, Eigen::Affine3d &T) {
    RansacFeatureSetMatcher ransac(0.0599, 0.9, 0.1, 0.6, 0.0499, false);
    Correspondences matches;
    OrientedPoint2D transform;
    double score_feature = ransac.matchSets(fixed.pts, moving.pts, transform, matches);
    ndt_feature::convertOrientedPoint2DToEigen(transform, T);
    return score_feature;
  }

  inline void getEstimatedCloudNDTFeatureFrames(const std::vector<NDTFeatureFrame> &frames, const Eigen::Affine3d &sensorPose, pcl::PointCloud<pcl::PointXYZ> &cloud) {
    for (size_t i = 0; i < frames.size(); i++) {
      Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T;
      T = frames[i].pose*sensorPose;
      
      pcl::PointCloud<pcl::PointXYZ> c = lslgeneric::transformPointCloud<pcl::PointXYZ>(T/*frames[i].pose*/, frames[i].pc);
      cloud += c;
    }
  }


} // namespace

#endif
