#include "ndt_feature/ndt_feature_link.h"
#include "covariance.hpp"
#include <iostream>

int main(){
	
// 	ndt_feature::NDTFeatureLink link;
	Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
// 	std::cout <<"Estimate " << link.getRelCov().inverse().format(cleanFmt) << std::endl;
	
	 Eigen::Vector2d eigenvec;
	 eigenvec << 1, 0;
	 std::pair<double, double> eigenval(10, 1);
	 
	 Eigen::Vector2d ortho = ndt_feature::getOrthogonalEigen(eigenvec);
	 std::cout << "Should be 0, 1 " << ortho.format(cleanFmt) << std::endl;
	 
	 Eigen::Matrix2d cov = ndt_feature::getCovarianceVec(eigenvec, eigenval);
	 std::cout << "Should be 10 0 | 0 1 ? " << std::endl << cov.format(cleanFmt) << std::endl;
	 
	 Eigen::Vector2d eigenvec2;
	 eigenvec2 << 1, 1;
	 std::pair<double, double> eigenval2(10, 1);
	 Eigen::Matrix2d cov2 = ndt_feature::getCovarianceVec(eigenvec2, eigenval2);
	 std::cout << "Should be 5.5  4.5 | 4.5  5.5 ? " << std::endl << cov2.format(cleanFmt) << std::endl;
	
}