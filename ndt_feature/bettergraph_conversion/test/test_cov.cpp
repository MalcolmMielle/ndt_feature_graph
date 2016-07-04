#include "ndt_feature/ndt_feature_link.h"


int main(){
	
	ndt_feature::NDTFeatureLink link;
	Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
	std::cout <<"Estimate " << link.getRelCov().inverse().format(cleanFmt) << std::endl;
	
}