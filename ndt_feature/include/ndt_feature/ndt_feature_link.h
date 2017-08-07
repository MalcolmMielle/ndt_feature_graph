#pragma once

#include <ndt_feature/interfaces.h>
//#include <ndt_feature/serialization.h>
#include <ndt_feature/utils.h>

namespace ndt_feature {

class NDTFeatureLink : public NDTFeatureLinkInterface {
public:
  NDTFeatureLink() {}
  NDTFeatureLink(size_t ref, size_t mov) : ref_idx(ref), mov_idx(mov) { 
    T.setIdentity();
    score = 0.;
  }
  NDTFeatureLink(const NDTFeatureLink& link){
	  ref_idx = link.getRefIdx();
	  mov_idx = link.getMovIdx();
	  T = link.getRelPose();
	  cov = getRelCov();
	  score = getScore();
  }
  size_t ref_idx; // Vector idx...
  size_t mov_idx;
  Eigen::Affine3d T; // From ref->mov.
  Eigen::Matrix3d cov;
  Eigen::MatrixXd cov_3d; // <-HACK for using the covairance returned by Matcher_D2D
  double score;

  // Interfaces
  virtual const Eigen::Affine3d& getRelPose() const {
    return T;
  }
  
  virtual const Eigen::Matrix3d& getRelCov() const {
    return cov;
  }
  
  virtual double getScore() const {
    return score;
  }

  virtual size_t getRefIdx() const {
    return ref_idx;
  }

  virtual size_t getMovIdx() const {
    return mov_idx;
  }

  void force2D() {
    ndt_feature::forceEigenAffine3dTo2dInPlace(this->T);
  }


};

} // namespace

// // Forward declaration of class boost::serialization::access
// namespace boost {
//   namespace serialization {
//     class access;
// 
//      template<typename Archive>
//     void serialize(Archive& ar, ndt_feature::NDTFeatureLink& o, const unsigned int version) {
//        ar & o.ref_idx & o.mov_idx & o.T & /* o.cov & */o.score;
//     }
//   }
// }
