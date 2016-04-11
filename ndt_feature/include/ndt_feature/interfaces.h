#pragma once

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

namespace ndt_feature {


class NDTFeatureNodeInterface {
public:
  // Access the vertex
  virtual const Eigen::Affine3d& getPose() const = 0;
  virtual const Eigen::Matrix3d& getCov() const = 0;
  virtual void setPose(const Eigen::Affine3d &pose) = 0;
  virtual void setCov(const Eigen::Matrix3d &cov) = 0;
};

class NDTFeatureLinkInterface {
public:
  virtual const Eigen::Affine3d& getRelPose() const = 0;
  virtual const Eigen::Matrix3d& getRelCov() const = 0;
  virtual double getScore() const = 0;
  virtual size_t getRefIdx() const = 0;
  virtual size_t getMovIdx() const = 0;

  friend std::ostream& operator<<(std::ostream &os, const NDTFeatureLinkInterface &obj)
  {
    os << "\n[" << obj.getRefIdx() << "<->" << obj.getMovIdx() << "] T: " << "["<<obj.getRelPose().translation()[0] << "," 
       << obj.getRelPose().translation()[1] << "]("
       << obj.getRelPose().rotation().eulerAngles(0,1,2)[2] << ")";
    os << "\n score : " << obj.getScore();
    return os;
  }
};

class NDTFeatureGraphInterface {
public:  
  virtual size_t getNbNodes() const = 0;
  virtual NDTFeatureNodeInterface& getNodeInterface(size_t idx) = 0;
  virtual const NDTFeatureNodeInterface& getNodeInterface(size_t idx) const = 0;
  
  
  virtual size_t getNbLinks() const = 0;
  virtual NDTFeatureLinkInterface& getLinkInterface(size_t idx) = 0;
  virtual const NDTFeatureLinkInterface& getLinkInterface(size_t idx) const = 0;
  
};

} // namespace
