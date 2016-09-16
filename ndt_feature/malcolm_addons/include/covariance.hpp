#ifndef NDTFEATURE_COVARIANCE_30082016
#define NDTFEATURE_COVARIANCE_30082016

#include "Eigen/Core"
#include <Eigen/LU>

namespace ndt_feature{
	
	
	/**
	 * @brief find orthogonal vector to vector
	 */
	Eigen::Vector2d getOrthogonalEigen(const Eigen::Vector2d& vec){
		Eigen::Vector2d out;
		out << -vec(1), vec(0);
		return out;
	}
	
	Eigen::Matrix2d getCovariance(const Eigen::Matrix2d& eigenvec, const std::pair<double, double>& eigenval){
		Eigen::Matrix2d eigenval_mat;
		eigenval_mat << eigenval.first, 0,
						0, eigenval.second;
// 		Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
// 		std::cout << "Eigan Val mat " << std::endl << eigenval_mat.format(cleanFmt) << std::endl;
		Eigen::Matrix2d eigenvec_inv = eigenvec.inverse();
// 		std::cout << "Eigan Vec inv " << std::endl << eigenvec_inv.format(cleanFmt) << std::endl;
		Eigen::Matrix2d covariance = eigenvec * eigenval_mat * eigenvec_inv;
// 		std::cout << "Cov " << std::endl << covariance.format(cleanFmt) << std::endl;
		return covariance;
		
	}
	
	/**
	 * @brief use the main eigen vector to find the orthogonal eigen vector and calculate the covariance associated
	 */
	Eigen::Matrix2d getCovarianceVec(const Eigen::Vector2d& eigenvec, const std::pair<double, double>& eigenval){
		Eigen::Vector2d ortho = getOrthogonalEigen(eigenvec);
// 		Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
// 		std::cout << "Ortho " << std::endl << ortho.format(cleanFmt) << std::endl;
		Eigen::Matrix2d eigenvec_mat;
		eigenvec_mat << eigenvec(0), ortho(0),
						eigenvec(1), ortho(1);
// 		std::cout << "Eigen vec mat " << std::endl << eigenvec_mat.format(cleanFmt) << std::endl;
		return getCovariance(eigenvec_mat, eigenval);
	}
	
	
	
	
	
}

#endif