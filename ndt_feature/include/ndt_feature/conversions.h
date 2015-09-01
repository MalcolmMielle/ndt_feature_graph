#ifndef NDT_FEATURE_CONVERSIONS_H
#define NDT_FEATURE_CONVERSIONS_H

#include <ndt_feature/flirtlib_utils.h>
#include <boost/foreach.hpp>
#include <Eigen/Eigen>
#include <ndt_map/ndt_map.h>
#include <ndt_map/cell_vector.h>

namespace ndt_feature {

  template <typename PointT>
    void addInterestPointSupportToPointCloud(const InterestPoint* ip, double varz, pcl::PointCloud<PointT> &pc) {
    
    const double x0 = ip->getPosition().x;
    const double y0 = ip->getPosition().y;
    
    const std::vector< Point2D > &support = ip->getSupport();
    
    PointT pt;
    size_t idx = 0;
    BOOST_FOREACH(const Point2D p, support)
      {
	pt.x = p.x;
	pt.y = p.y;
	pt.z = varz*((double)rand())/(double)INT_MAX;
	pc.points.push_back(pt);
      }
  }
  
  template <typename PointT>
    void convertInterestPointVecToPointClouds(const InterestPointVec &pts, double varz, pcl::PointCloud<PointT> &pc,  std::vector<std::vector<size_t> > &indices) {
    
    pc.clear();
    indices.clear();
    
    size_t idx = 0;
    BOOST_FOREACH(const InterestPoint* p, pts) 
      {
	addInterestPointSupportToPointCloud(p, varz, pc);
	// This looks a bit weird... indices holds all points here - to be able to use the existing ndt_map API.
	std::vector<size_t> inner_idx;
	for (size_t i = 0; i < pc.points.size(); i++) {
	  inner_idx.push_back(idx);
	  idx++;
	}
	indices.push_back(inner_idx);
      }
    
  }
  
    void addInterestPointToCellVectorFixedCov(const InterestPoint &pt,
						      const Eigen::Matrix3d &cov,
						      lslgeneric::CellVector* cv) {
    lslgeneric::NDTCell* ndt_cell = new lslgeneric::NDTCell();
    ndt_cell->setCov(cov);
    ndt_cell->setMean(Eigen::Vector3d(pt.getPosition().x, pt.getPosition().y, 0.));
    cv->addNDTCell(ndt_cell);
  }
  
    void addInterestPointsToCellVectorFixedCov(const InterestPointVec &pts,
						       const Eigen::Matrix3d &cov,
						       lslgeneric::CellVector* cv) {
    for (size_t i = 0; i < pts.size(); i++) {
      addInterestPointToCellVectorFixedCov(*pts[i], cov, cv);
    }
  }
  
  void convertCorrespondencesToCellvectorsFixedCovWithCorr(const Correspondences &matches,
							     const Eigen::Matrix3d &cov, 
							     lslgeneric::CellVector *source,
							     lslgeneric::CellVector *target,
							     std::vector<std::pair<int, int> > &corr) {
    
    size_t idx = 0;
    BOOST_FOREACH(const Correspondence& c, matches) 
      {
	addInterestPointToCellVectorFixedCov(*c.first, cov, source);
	addInterestPointToCellVectorFixedCov(*c.second, cov, target);
	corr.push_back(std::pair<int, int>(idx, idx));
	idx++;
      }
  }
  

    
} // namespace

#endif
