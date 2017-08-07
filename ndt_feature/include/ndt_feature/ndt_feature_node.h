#pragma once

#include <ndt_feature/interfaces.h>
#include <ndt_feature/ndt_feature_fuser_hmt.h>
#include <pcl/point_cloud.h>
#include <ndt_map/pointcloud_utils.h>


// For serialization...
#include <iostream>
// #include <semrob_generic/serialization.h>
// #include <boost/archive/text_oarchive.hpp>
// #include <boost/archive/text_iarchive.hpp>
// #include <pcl/io/pcd_io.h>

namespace ndt_feature {

inline bool saveAffine3d(const Eigen::Affine3d &T, const std::string& fileName) {
//   std::ofstream ofs(fileName.c_str());
//   if (ofs.fail())
//     return false;
//   boost::archive::text_oarchive ar(ofs);
//   ar & T;
//   return true;
}
 
inline bool loadAffine3d(Eigen::Affine3d &T, const std::string& fileName) {
//   std::ifstream ifs(fileName.c_str());
//   if (!ifs.is_open())
//     return false;
//   
//   boost::archive::text_iarchive ar(ifs);
//   ar & T;
//   return true;
}

//! Class to hold a node - a fused map with features along with a global pose offset estimate. To be used in an optimization framework
class NDTFeatureNode : public NDTFeatureNodeInterface {

public:
	//TODO : would initialize NDTFeatureHMT in here prove more useful ? For now the user is forced to init everything itself otherwise you can't access the Eigen matrix straight away. Maybe a combination of a pointer destructor in the destructor and some setter getter ? I think it will be better for memory allocation also.
  NDTFeatureNode() : nbUpdates(0), map(NULL) {
    T.setIdentity();
    Tlocal_odom.setIdentity();
    Tlocal_fuse.setIdentity();
  }
  
  //Bugging Henrik code
//   NDTFeatureNode(const NDTFeatureNode& ndt_feat){
// 	  cov = ndt_feat.getCov();
// 	  T = ndt_feat.getPose();
// 	  Tlocal_odom = ndt_feat.getTLocalOdom();
// 	  Tlocal_fuse = ndt_feat.getTLocalFuse();
// 	  pts = ndt_feat.getPts();
// 	  map = new NDTFeatureFuserHMT(ndt_feat.map->getParam());
// 	  *map = *(ndt_feat.map);
//   }

  ~NDTFeatureNode() {
    std::cerr << "FeatureNode Destructor" << std::endl;
    std::cerr << "FeatureNode Destructor - done" << std::endl;
  }
  
  void copyNDTFeatureNode(const NDTFeatureNode& ndt_feat){
	  cov = ndt_feat.getCov();
	  T = ndt_feat.getPose();
	  Tlocal_odom = ndt_feat.getTLocalOdom();
	  Tlocal_fuse = ndt_feat.getTLocalFuse();
	  pts = ndt_feat.getPts();
	  map = new NDTFeatureFuserHMT(ndt_feat.map->getParam());
	  //TODO :
	  *map = *(ndt_feat.map);
  }

  NDTFeatureFuserHMT* map;
  Eigen::Affine3d T;
  Eigen::Matrix3d cov;

  Eigen::Affine3d Tlocal_odom; // Incremental odometry between successive local maps.
  Eigen::Affine3d Tlocal_fuse; // Incremental fuse estimates between sucessive local maps.
  pcl::PointCloud<pcl::PointXYZ> pts; // Only for visualizaion purposes...
  int nbUpdates;
  
  double time_last_update;
  

void addCloud(Eigen::Affine3d &T, const pcl::PointCloud<pcl::PointXYZ> &pc) {
    this->pts += lslgeneric::transformPointCloud(T,pc);
}

pcl::PointCloud<pcl::PointXYZ> getGlobalPointCloud() {
return lslgeneric::transformPointCloud<pcl::PointXYZ>(this->T, this->pts);
}

pcl::PointCloud<pcl::PointXYZ> getLocalPointCloud() {
return this->pts;
}

  // Specific save function, use the ndt map saving function.
  bool save(const std::string &fileName) {
    std::string nd_file = fileName;
    if (!map->save(nd_file)) {
      return false;
    }
    std::string T_file = fileName + ".T";
    if (!saveAffine3d(T, T_file)) {
      return false;
    }
    std::string Tlocal_odom_file = fileName + "local_odom.T";
    if (!saveAffine3d(Tlocal_odom, Tlocal_odom_file)) {
      return false;
    }
    std::string Tlocal_fuse_file = fileName + "local_fuse.T";
    if (!saveAffine3d(Tlocal_fuse, Tlocal_fuse_file)) {
      return false;
    }

    std::string pc_file = fileName + ".pcd";
    if (!this->pts.empty()) {
      pcl::io::savePCDFileASCII (pc_file, this->pts);
    }
    return true;
  }
  
  bool load(const std::string &fileName) {

if (!map->load(fileName)) {
      return false;
    }

    std::string T_file = fileName + ".T";
    if (!loadAffine3d(T, T_file)) {
      return false;
    }
    std::string Tlocal_odom_file = fileName + "local_odom.T";
    if (!loadAffine3d(Tlocal_odom, Tlocal_odom_file)) {
      return false;
    }
    std::string Tlocal_fuse_file = fileName + "local_fuse.T";
    if (!loadAffine3d(Tlocal_fuse, Tlocal_fuse_file)) {
      return false;
    }
    

    std::string pc_file = fileName + ".pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pc_file, this->pts) == -1)
    {
      //      return false;
    }
    return true;
  }

  NDTFeatureMap& getFeatureMap() {
    assert(map != NULL);
    return map->getFeatureMap();
  }

  const NDTFeatureMap& getFeatureMap() const {
    assert(map != NULL);
    return map->getFeatureMap();
  }

  lslgeneric::NDTMap& getNDTMap() {
    assert(map != NULL);
    assert(map->map != NULL);
    return *(map->map);
  }

  const NDTFeatureFuserHMT& getFuser() const {return *map;}
  NDTFeatureFuserHMT& getFuser() {return *map;}
  
  
  const lslgeneric::NDTMap& getNDTMap() const { 
    assert(map != NULL);
    assert(map->map != NULL);
    return *(map->map);
  }

  // Interface
  virtual const Eigen::Affine3d& getPose() const {
    return T;
  }
  virtual const Eigen::Matrix3d& getCov() const {
    return cov;
  }
  virtual const Eigen::Affine3d& getTLocalOdom() const {
    return Tlocal_odom;
  }
  virtual const Eigen::Affine3d& getTLocalFuse() const {
    return Tlocal_fuse;
  }
  virtual const pcl::PointCloud<pcl::PointXYZ>& getPts() const {
	return pts;
  }
  virtual pcl::PointCloud<pcl::PointXYZ>& getPts() {
	pcl::PointCloud<pcl::PointXYZ>& pt(pts);
	return pt;
  }

  void setPose(const Eigen::Affine3d &pose) { T = pose; }
  void setCov(const Eigen::Matrix3d &cov_) { cov = cov_; }

  void force2D() {
    ndt_feature::forceEigenAffine3dTo2dInPlace(this->T);
    ndt_feature::forceEigenAffine3dTo2dInPlace(this->Tlocal_odom);
    ndt_feature::forceEigenAffine3dTo2dInPlace(this->Tlocal_fuse);
  }
};


// Matching functions
inline double overlapNDTOccupancyScore(NDTFeatureNode &ref, NDTFeatureNode &mov, const Eigen::Affine3d &T) {
  // Use T to move the mov frame center... return a score based on the number of cells that matched/total number of cells.
  
  std::vector<lslgeneric::NDTCell*> mov_vector=mov.map->map->getAllInitializedCells();

  double diff_sum = 0.;
  size_t nb_sum = 0;
  
  for (int cell_idx=0;cell_idx<mov_vector.size();cell_idx++){
    pcl::PointXYZ pt = mov_vector[cell_idx]->getCenter();
    double mov_occ = mov_vector[cell_idx]->getOccupancyRescaled();
    if (mov_occ == 0.5) // Iniitated but don't have any readings...
      continue;
    // Transform the point.
    Eigen::Vector3d ept(pt.x, pt.y, pt.z);
    Eigen::Vector3d tpt = T*ept;
    pt.x = tpt[0]; pt.y = tpt[1]; pt.z = tpt[2];

    // Check the corresponding occupancy value (if any)
    lslgeneric::NDTCell* cell;
    if (ref.map->map->getCellForPoint(pt, cell, false)) { 
      if (cell != NULL) {
        // false - dont check for gaussian (need only to check for the occuancy value here...
        double ref_occ = cell->getOccupancyRescaled();
        if (ref_occ != 0.5) {
          nb_sum++;
          double diff = (mov_occ - ref_occ);
          diff_sum += diff*diff;
        }
      }
    }
  }
  for (size_t i = 0; i < mov_vector.size(); i++) {
    delete mov_vector[i];
  }
  if (nb_sum == 0)
    return 1.;

  return diff_sum/(1.*nb_sum);
}

inline double matchNodesUsingFeatureMap(const NDTFeatureNode &ref, const NDTFeatureNode &mov, Correspondences &matches, Eigen::Affine3d &T) 
{
  return matchFeatureMap(ref.map->featuremap, mov.map->featuremap, matches, T);
}

} // namespace
