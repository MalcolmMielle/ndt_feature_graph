#include "ndt_feature/ndt_feature_graph.h"


int main(){
	
	ndt_feature::NDTFeatureFuserHMT::Params fuser_params;
	ndt_feature::NDTFeatureFuserHMT* fuser = new ndt_feature::NDTFeatureFuserHMT(fuser_params);
	
	Eigen::Affine3d initPos;
// 	initPos.matrix() << 0, 0, 0, ;
	pcl::PointCloud<pcl::PointXYZ> cloudOrig;
	const InterestPointVec pts; 
	fuser->initialize(initPos, cloudOrig, pts);
	
	ndt_feature::NDTFeatureFuserHMT fuser2(*fuser);
	
	delete fuser;
	
	
}