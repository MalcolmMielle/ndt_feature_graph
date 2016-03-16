#pragma once

#include <ndt_feature/ndt_feature_fuser_hmt.h>

// For serialization...
#include <iostream>
#include <semrob_generic/serialization.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

namespace ndt_feature {

bool saveAffine3d(const Eigen::Affine3d &T, const std::string& fileName) {
  std::ofstream ofs(fileName.c_str());
  if (ofs.fail())
    return false;
  boost::archive::text_oarchive ar(ofs);
  ar & T;
  return true;
}
 
bool loadAffine3d(Eigen::Affine3d &T, const std::string& fileName) {
  std::ifstream ifs(fileName.c_str());
  if (!ifs.is_open())
    return false;
  
  boost::archive::text_iarchive ar(ifs);
  ar & T;
  return true;
}

//! Class to hold a node - a fused map with features along with a global pose offset estimate. To be used in an optimization framework
class NDTFeatureNode {

public:
  NDTFeatureFuserHMT* map;
  Eigen::Affine3d T;

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
  }
  
  bool load(const std::string &fileName) {
    if (!map->load(fileName)) {
      return false;
    }
    std::string T_file = fileName + ".T";
    if (!loadAffine3d(T, T_file)) {
      return false;
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

  const lslgeneric::NDTMap& getNDTMap() const { 
    assert(map != NULL);
    assert(map->map != NULL);
    return *(map->map);
  }
};


// Matching functions
double overlapNDTOccupancyScore(NDTFeatureNode &ref, NDTFeatureNode &mov, const Eigen::Affine3d &T) {
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
          //std::cout << "mov_occ : " << mov_occ << " ref_occ : " << cell->getOccupancyRescaled() << std::endl;
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

double matchNodesUsingFeatureMap(const NDTFeatureNode &ref, const NDTFeatureNode &mov, Correspondences &matches, Eigen::Affine3d &T) 
{
  return matchFeatureMap(ref.map->featuremap, mov.map->featuremap, matches, T);
}

} // namespace
