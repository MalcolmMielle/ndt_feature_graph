
#pragma once

#include <flirtlib_ros/flirtlib.h>
#include <flirtlib_ros/conversions.h>
#include <ndt_feature/utils.h>
#include <ndt_feature/flirtlib_utils.h>

// For serialization...
#include <iostream>
// #include <ndt_feature/serialization.hpp>
// #include <boost/archive/binary_iarchive.hpp>
// #include <boost/archive/binary_oarchive.hpp>
// #include <boost/archive/text_oarchive.hpp>
// #include <boost/archive/text_iarchive.hpp>

namespace ndt_feature {

inline bool saveInterestPointVec(const InterestPointVec &pts, const std::string &fileName) {
//   std::ofstream ofs(fileName.c_str());
//   boost::archive::binary_oarchive ar(ofs);
//   ar & pts;
//   // size_t s = pts.size();
//   // ar & s;
//   // for (size_t i = 0; i < s; i++) {
//   //   ar & *(pts[i]);
//   // }
//   ofs.close();
//   return true; // Check for throws.
}

inline bool loadInterestPointVec(InterestPointVec &pts, const std::string &fileName) {
//   std::ifstream ifs(fileName.c_str());
//   boost::archive::binary_iarchive ar(ifs);
//   ar & pts;
//   // size_t s;
//   // ar & s;
//   // std::cout << "Loading : " << s << " feature ...." << std::endl;
//   // for (size_t i = 0; i < s; i++) {
//   //   InterestPoint* pt = new InterestPoint();
//   //   ar & *(pt);
//   //   pts.push_back(pt);
//   // }
//   // std::cout << "... done." << std::endl;
//   ifs.close();
//   return true; // Check for throws.
}


//! Class to hold features and to handle statistics in order to convert them into NDTCell type structures.
class NDTFeatureMap {
public:
  NDTFeatureMap() { counter_ = 0; }

  ~NDTFeatureMap() {
    std::cerr << "NDTFeatureMap Destructor" << std::endl;
    for (size_t i = 0; i < map.size(); i++) {
      delete map[i];
    }
    std::cerr << "NDTFeatureMap Destructor - done" << std::endl;
  }
  void update(InterestPointVec &pts) {
    // For now simply append this to the map.
    if (counter_ % 4 == 0) {
      map.insert(map.end(), pts.begin(), pts.end());
    }
    counter_++;
  }

  InterestPointVec& getMap() { return map; }

  bool save(const std::string &fileName) {
    return saveInterestPointVec(map, fileName);
  }
  
  bool setDistFunction(const HistogramDistance<double>* distFunction) {
    if (distFunction == NULL)
      return false;

    // Assign the distance function pointer
    bool ret = true;
    BOOST_FOREACH (InterestPoint* p, map) {
      BetaGrid *beta_grid = dynamic_cast<BetaGrid *>(p->getDescriptor());
      if (beta_grid != NULL)
        beta_grid->setDistanceFunction(distFunction);
      else
        ret = false;
    }
    return ret;
  }

  bool load(const std::string &fileName) {
    bool ret = loadInterestPointVec(map, fileName);

    return ret;
  }

  InterestPointVec map;
private:
  int counter_;
};


inline double matchFeatureMap(const NDTFeatureMap &ref, const NDTFeatureMap &mov, Correspondences &matches, Eigen::Affine3d &T) 
{
  boost::shared_ptr<RansacFeatureSetMatcher> ransac_(new RansacFeatureSetMatcher(0.0599, 0.9, 0.1, 0.6, 0.0499, false));

  if (ref.map.empty() || mov.map.empty()) {
    return std::numeric_limits<double>::max();
  }

  //  RansacFeatureSetMatcher ransac(0.0599, 0.9, 0.1, 0.6, 0.0499, false);
  OrientedPoint2D transform;
  double score = ransac_->matchSets(ref.map, mov.map, transform, matches);

  if (!validOrientedPoint2D(transform)) { // the transform could be nan
    return std::numeric_limits<double>::max();
  }
  ndt_feature::convertOrientedPoint2DToEigen(transform, T);
  ndt_feature::printTransf(T);
  return score;
}

}

