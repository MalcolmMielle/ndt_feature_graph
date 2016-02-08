#pragma once

#include <ndt_registration/ndt_matcher_d2d.h>
#include <ndt_registration/ndt_matcher_d2d_feature.h>
#include <ndt_feature/ndt_matcher_d2d_fusion.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/cell_vector.h>
#include <ndt_map/lazy_grid.h>
#include <ndt_map/pointcloud_utils.h>

namespace lslgeneric {

Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> ICPwithCorrMatch(lslgeneric::NDTMap &targetNDT, lslgeneric::NDTMap &sourceNDT, const std::vector<std::pair<int,int> > &corresp) {
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T;
    T.setIdentity();

    // This only works with cell vectors
    

    Eigen::Vector3d c0;
    Eigen::Vector3d c1;
    Eigen::Matrix3d H;
    {
        c0.setZero();
        c1.setZero();

        std::cout << "1" << std::endl;
        
        // get centroids
        for (size_t i = 0; i < corresp.size(); i++) {
          c0 += targetNDT.getCellIdx(corresp[i].first)->getMean();
          c1 += sourceNDT.getCellIdx(corresp[i].second)->getMean();
        }
        c0 *= (1./(1.*corresp.size()));
        c1 *= (1./(1.*corresp.size()));
        std::cout << "1" << std::endl;
        std::cout << "c0 : " << c0 << std::endl;
        std::cout << "c1 : " << c1 << std::endl;
        
        Eigen::MatrixXd target(corresp.size(), 3);
        std::cout << "target : " << target << std::endl;
        Eigen::MatrixXd source(corresp.size(), 3);
        for (size_t i = 0; i < corresp.size(); i++) {
            target.row(i) = targetNDT.getCellIdx(corresp[i].first)->getMean() - c0;
            source.row(i) = sourceNDT.getCellIdx(corresp[i].second)->getMean() - c1;
        }

        std::cout << "target : " << target << std::endl; 
        std::cout << "source : " << source << std::endl;
        std::cout << "target * source.transpose() : " << target  * source.transpose() << std::endl;

        H = target.transpose() * source;

        std::cout << "H : " << H << std::endl;
    }
    
    // do the SVD thang
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d R = V * svd.matrixU().transpose();
    double det = R.determinant();
    //ntot++;
    if (det < 0.0)
    {
        //nneg++;
        V.col(2) = V.col(2)*-1.0;
        R = V * svd.matrixU().transpose();
    }
    Eigen::Vector3d tr = c0-R.transpose()*c1;    // translation
    
    std::cout << "translation : " << c0 - c1 << std::endl;
    std::cout << "tr : " << tr << std::endl;
    std::cout << "c0-R.transpose()*c1 : " << c0-R.transpose()*c1 << std::endl;
    // transformation matrix, 3x4
    Eigen::Matrix<double,3,4> tfm;
    tfm.block<3,3>(0,0) = R.inverse();
    tfm.col(3) = tr;
    T = tfm;

    return T;
}

void printTransf(const Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T) {
  std::cout << "translation "<<T.translation().transpose()
             <<" (norm) "<<T.translation().norm()<<std::endl;
  std::cout << "rotation " <<T.rotation().eulerAngles(0,1,2).transpose()
             <<" (norm) "<<T.rotation().eulerAngles(0,1,2).norm()<<std::endl;
}

void printTransf2d(const Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T) {
  std::cout << "["<<T.translation()[0] << "," 
            << T.translation()[1] << "]("
            << T.rotation().eulerAngles(0,1,2)[2] << ")" << std::endl;
}


void addNDTCellToMap(NDTMap* map, NDTCell* cell) {
  CellVector* idx = dynamic_cast<CellVector*> (map->getMyIndex());
  if (idx != NULL) {
    NDTCell* nd = (NDTCell*)cell->clone();
    idx->addNDTCell(cell);
  }
}


Eigen::Vector3d computeLocalCentroid(const Eigen::Vector3d &map_centroid, const Eigen::Vector3d &local_pos, double resolution) {
  Eigen::Vector3d diff = map_centroid - local_pos;
  std::cout << "local_pos : " << local_pos << std::endl;
  std::cout << "map_centroid : " << map_centroid << std::endl;
  std::cout << "diff : " << diff << std::endl;
  Eigen::Vector3d local_centroid;
  for (int i = 0; i < 3; i++) {
    local_centroid(i) = diff(i) - floor(diff(i) / resolution) * resolution;
    std::cout << "floor... : " << floor(diff(i)/resolution) << std::endl;
  }
  std::cout << "local_centroid : " << local_centroid << std::endl;

  return local_centroid;
}



} // namespace
