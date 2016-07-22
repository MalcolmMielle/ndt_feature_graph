#ifndef NDTFEATURE_CONVERSION_22072016
#define NDTFEATURE_CONVERSION_22072016

#include "ndt_feature/ndt_feature_graph.h"
#include "G2OGraphMaker.hpp"


namespace ndt_feature{
	
	inline ndtGraphToG2O(const ndt_feature::NDTFeatureGraph& ndt_graph, G2OGraphMarker& g2o_graph){
		
		addRobotPoses(ndt_graph, g2o_graph);
		addOdometry(ndt_graph, g2o_graph);
		
	}
	
	inline addRobotPoses(const ndt_feature::NDTFeatureGraph& ndt_graph, G2OGraphMarker& g2o_graph){
		
// 		int size_before = _optimizer.vertices().size();
			 
		for (size_t i = 0; i < ndt_graph.getNbNodes(); ++i) {
			std::cout << "Adding node " << i << std::endl;
			NDTFeatureNode* feature = new NDTFeatureNode();
			feature->copyNDTFeatureNode( (const NDTFeatureNode&)ndt_graph.getNodeInterface(i) );
			g2o::VertexSE2* robot =  new g2o::VertexSE2;
			//ATTENTION: make sure that id is good
			robot->setId(i);
			ndtFeatureNodeToVertexSE2(*feature, *robot);
			std::cout << "Robot " << robot->estimate().toVector() << std::endl;
			g2o_graph.addRobotPose(robot);			
		}
		
// 		//Make sure the convertion worked
// 		if(ndt_graph.getNbNodes() != _optimizer.vertices().size() - size_before ){
// 			std::cout << "Merde" << std::endl;
// 		}
// 		assert(ndt_graph.getNbNodes() == ( _optimizer.vertices().size() - size_before ) );
// 		std::cout << " DONE WITH VERT " << _optimizer.vertices().size() << std::endl;
// 		std::cout << "SAVE now" << std::endl;
	}
	
	
	inline void addOdometry(const ndt_feature::NDTFeatureGraph& ndt_graph, G2OGraphMarker& g2o_graph){
			
		auto links = ndt_graph.getOdometryLinks();
			
		for(auto tmp = 0 ; tmp < links.size() ; ++tmp){
			Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
			std::cout <<"Estimate before anything " << links[tmp].getRelCov().inverse().format(cleanFmt) << std::endl;
			std::cout << "Adding Edge" << std::endl;	
		}
		
// 				graph.appendLinks(links_odom);
		std::cout << ndt_graph.getNbNodes() - 1 << " ? ==" <<  links.size() << std::endl;
		assert(ndt_graph.getNbNodes() - 1 == links.size() );
		
		for (size_t i = 0 ; i <links.size() ; ++i) {
			Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
			std::cout <<"Estimate before anything " << links[i].getRelCov().inverse().format(cleanFmt) << std::endl;
			std::cout << "Adding Edge" << std::endl;
// 				NDTFeatureLink link = NDTFeatureLink((const NDTFeatureLink&) graph.getLinkInterface(i));
			g2o::EdgeSE2* odometry = new g2o::EdgeSE2;
			ndtFeatureLinkToEdgeSE2(links[i], *odometry, g2o_graph);
			g2o_graph.addOdometry(odometry);
		} 
		
	}
	
	inline void ndtFeatureLinkToEdgeSE2(const NDTFeatureLink& link, g2o::EdgeSE2& edge, bool useCov, G2OGraphMarker& g2o_graph)
	{
		size_t from = link.getRefIdx() ;
		size_t toward = link.getMovIdx() ;
		
		std::cout << "from " << from << " toward " << toward << std::endl;
		
		edge.vertices()[0] = g2o_graph.getVertex(from);
		edge.vertices()[1] = g2o_graph.getVertex(toward);
		
		Eigen::Affine3d affine = link.getRelPose();		
		Eigen::Isometry2d isometry2d = affine3d2Isometry2d(affine);
// 		double x = cumulated_translation(0, 3);
// 		double y = cumulated_translation(1, 3);
		g2o::SE2 se2(isometry2d);
		
		edge.setMeasurement(se2);
		
		//HOW IT SHOUDL BE DONE :
// 		//Information is covariance
// 		Eigen::Matrix3d cov = link.getRelCov().inverse();
// 		
// 		Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "\t", " ", "", "", "", "");
// 		Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
// 		std::cout <<"Estimate " << cov.format(cleanFmt) << std::endl;
// 		std::cout <<"Estimate " << link.getRelCov().inverse().format(cleanFmt) << std::endl;
		
		
		//BUG : Covariance of each node : USE SCAN MATCHING
// 		auto cov_fo_node = node.map.getCov();
		
		Eigen::Matrix3d covariance;
		if(useCov == false){
			//HACK ? : HOW IT'S DONE IN G2o. Also maybe look here : https://robotics.stackexchange.com/questions/7960/how-to-generate-edges-for-pose-graph-slam-from-known-positions-and-loops
			Eigen::Vector2d transNoise(0.05, 0.01);
			//Deg to radian
			double rotNoise = 2. * 0.01745329251994329575;

			
			covariance.fill(0.);
			
			std::cout <<"Estimate of zero " << covariance.format(cleanFmt) << std::endl;
			covariance(0, 0) = transNoise[0]*transNoise[0];
			covariance(1, 1) = transNoise[1]*transNoise[1];
			covariance(2, 2) = rotNoise*rotNoise;
			
			std::cout <<"Estimate of covariance " << covariance.format(cleanFmt) <<  std::endl << " With rot " << rotNoise*rotNoise << std::endl;
		}
		else{
			covariance = link.getRelCov();
		}
		Eigen::Matrix3d information = covariance.inverse();
		
// 		Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "\t", " ", "", "", "", "");
		
		std::cout <<"Estimate of information " << information.format(cleanFmt) << std::endl;
// 		std::cout <<"Estimate " << link.getRelCov().inverse().format(cleanFmt) << std::endl;

		edge.setInformation(information);

	}
	
	inline void ndtFeatureNodeToVertexSE2(const NDTFeatureNode& feature, g2o::VertexSE2& vertex_se2)
	{
		Eigen::Affine3d affine = Eigen::Affine3d(feature.getPose());
// 		getRelPose();		
		Eigen::Isometry2d isometry2d = affine3d2Isometry2d(affine);
		
		Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "\t", " ", "", "", "", "");
		std::cout <<"Estimate " << isometry2d.matrix().format(cleanFmt) << std::endl;
// 		double x = cumulated_translation(0, 3);
// 		double y = cumulated_translation(1, 3);
		g2o::SE2 se2(isometry2d);
		
		std::cout << "SE2 " << se2.toVector() << std::endl;
		vertex_se2.setEstimate(se2);
		
		std::cout << "Vector in func " << vertex_se2.estimate().toVector() << std::endl;
	}
	
	inline Eigen::Isometry2d affine3d2Isometry2d(const Eigen::Affine3d& affine){
		
		Eigen::Affine2d affine2d = lslgeneric::eigenAffine3dTo2d(affine);
		Eigen::Isometry2d isometry2d;
		isometry2d.translation() = affine2d.translation();
		isometry2d.linear() = affine2d.rotation();
		return isometry2d;
		
	}
	
	
	
}