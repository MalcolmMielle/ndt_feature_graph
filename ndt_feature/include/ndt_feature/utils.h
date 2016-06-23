#pragma once

#include <ndt_registration/ndt_matcher_d2d.h>
#include <ndt_registration/ndt_matcher_d2d_feature.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/cell_vector.h>
#include <ndt_map/lazy_grid.h>
#include <ndt_map/pointcloud_utils.h>

namespace lslgeneric {



inline double getRobustYawFromAffine3d(const Eigen::Affine3d &a) {
  // To simply get the yaw from the euler angles is super sensitive to numerical errors which will cause roll and pitch to have angles very close to PI...
  Eigen::Vector3d v1(1,0,0);
  Eigen::Vector3d v2 = a.rotation()*v1;
  double dot = v1(0)*v2(0)+v1(1)*v2(1); // Only compute the rotation in xy plane...
  double angle = acos(dot);
  // Need to find the sign
  if (v1(0)*v2(1)-v1(1)*v2(0) > 0)
    return angle;
  return -angle;
}

inline void distanceBetweenAffine3d(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2, double &dist, double &angularDist) {
  Eigen::Affine3d tmp = (p1.inverse() * p2);
  dist = tmp.translation().norm();
  //  angularDist = tmp.rotation().eulerAngles(0,1,2).norm();
  angularDist = fabs(getRobustYawFromAffine3d(tmp));
}

inline Eigen::Affine2d eigenAffine3dTo2d(const Eigen::Affine3d &a3d) {
  return Eigen::Translation2d(a3d.translation().topRows<2>()) *
    Eigen::Rotation2D<double>(getRobustYawFromAffine3d(a3d));//a3d.linear().topLeftCorner<2,2>();
    
}

inline Eigen::Affine3d eigenAffine2dTo3d(const Eigen::Affine2d &a2d) {
  //  Eigen::Rotation2D<double> rot = Eigen::Rotation2D<double>::fromRotationMatrix(a2d.rotation());//Eigen::fromRotationMatrix(a2d.translation());
  double angle = atan2(a2d.rotation()(1,0), a2d.rotation()(0,0));//rot.angle();//acosa2d.rotation()(0,1)/a2d.rotation()(0,0);
  return Eigen::Translation3d(a2d.translation()(0), a2d.translation()(1), 0.) *
    Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
}

inline Eigen::Affine3d forceEigenAffine3dTo2d(const Eigen::Affine3d &a3d) {
  return eigenAffine2dTo3d(eigenAffine3dTo2d(a3d));
}

inline void forceEigenAffine3dTo2dInPlace(Eigen::Affine3d &a3d) {
  Eigen::Affine3d t = a3d;
  a3d = forceEigenAffine3dTo2d(a3d);
}

inline Eigen::Affine2d getAffine2d(double x, double y, double th) {
  return Eigen::Translation2d(x,y) *
    Eigen::Rotation2D<double>(th);
}

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
    //<< T.rotation().eulerAngles(0,1,2)[2] << ")" << std::endl;
            << lslgeneric::getRobustYawFromAffine3d(T) << ")" << std::endl;
    }


void convertAffineToVector(const Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T,
                                Eigen::Matrix<double,6,1> &vec) {
  vec[0] = T.translation().transpose()[0];
  vec[1] = T.translation().transpose()[1];
  vec[2] = T.translation().transpose()[2];
  vec[3] = T.rotation().eulerAngles(0,1,2)[0];
  vec[4] = T.rotation().eulerAngles(0,1,2)[1];
  vec[5] = T.rotation().eulerAngles(0,1,2)[2];
}

void addNDTCellToMap(NDTMap* map, NDTCell* cell) {
  CellVector* idx = dynamic_cast<CellVector*> (map->getMyIndex());
  if (idx != NULL) {
    NDTCell* nd = (NDTCell*)cell->copy();

    idx->addNDTCell(nd);
  }
  else {
    std::cerr << "Only cellvector implemented" << std::endl;
  }
}

//! Note: this will overwrite any existing cells
void setNDTCellToMap(NDTMap *map, NDTCell* cell) {
  LazyGrid* lz = dynamic_cast<LazyGrid*> (map->getMyIndex());
  if (lz != NULL) {
    NDTCell* nd = (NDTCell*)cell->clone();
    // Is there already a cell here?
    NDTCell* c = NULL;
    
    Eigen::Vector3d mean = cell->getMean();
    pcl::PointXYZ pt_mean;
    pt_mean.x = mean[0]; pt_mean.y = mean[1]; pt_mean.z = mean[2];
    //    lz->getCellAt(pt_mean,c);
    c = lz->getCellForPoint(pt_mean);
    if (c == NULL) {
      // Needs some means to create it... do this by adding a point the returned cell will be an nice an allocated one.
      c = lz->addPoint(pt_mean);
    }
    //    c->setParameters(0.1, 8*M_PI/18, 1000); //Increase how small the covariance could be.
    c->setMean(cell->getMean());
    c->setCov(cell->getCov());
  }
  else {
    std::cerr << "Only lazygrid implemented..." << std::endl;
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

bool discardCell(NDTMap &map, const pcl::PointXYZ &pt) {
  NDTCell *cell;
  if (map.getCellAtPoint(pt, cell)) {
    cell->hasGaussian_ = false;
    return true;
  }
  return false;
}

// int discardDistantCells(NDTMap &map, const pcl::PointCloud<pcl::PointXYZ> &pts, double max_dist) {
//   for (size_t i = 0; i < pts.size(); i++) {
//     NDTCell *cell;
//     if (map.getCellAtPoint(pts[i], cell)) {
      
//     }
    
//   }
// }

std::string transformToEvalString(const Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T) {
  std::ostringstream stream;
  stream << std::setprecision(std::numeric_limits<double>::digits10);
  Eigen::Quaternion<double> tmp(T.rotation());
  stream << T.translation().transpose() << " " << tmp.x() << " " << tmp.y() << " " << tmp.z() << " " << tmp.w() << std::endl; 
  return stream.str();
}

std::string transformToEval2dString(const Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T) {
  std::ostringstream stream;
  stream << std::setprecision(std::numeric_limits<double>::digits10);
  double yaw = getRobustYawFromAffine3d(T);
  Eigen::Affine3d T2 = Eigen::Translation<double,3>(T.translation()[0], T.translation()[1], 0.) * Eigen::AngleAxis<double>(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaternion<double> tmp(T2.rotation());
  stream << T2.translation()[0] << " " << T2.translation()[1] << " 0. " << " " << tmp.x() << " " << tmp.y() << " " << tmp.z() << " " << tmp.w() << std::endl; 
  return stream.str();
}


} // namespace
