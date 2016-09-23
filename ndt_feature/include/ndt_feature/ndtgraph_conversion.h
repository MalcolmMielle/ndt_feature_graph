#ifndef NDT_FEATURE_CONVERSIONS_NDT_GRAPH_22102016

#define NDT_FEATURE_CONVERSIONS_NDT_GRAPH_22102016

#include "ndt_feature_graph.h"
#include "ndt_feature/NDTEdgeMsg.h"
#include "ndt_feature/NDTNodeMsg.h"
#include "ndt_feature/NDTGraphMsg.h"
#include "ndt_feature/NDTFeatureFuserHMTMsg.h"
#include "eigen_conversions/eigen_msg.h"


namespace ndt_feature {
	
	
	
	void edgeToMsg(const NDTFeatureLink& link, ndt_feature::NDTEdgeMsg& m){
		m.ref_idx = link.ref_idx;
		m.mov_idx = link.mov_idx;
		geometry_msgs::Pose pose;
		tf::poseEigenToMsg (link.T, pose);
		m.T = pose;
// 		float64[] cov
		std_msgs::Float64MultiArray arr;
		tf::matrixEigenToMsg(link.cov, arr);
		m.cov = arr;
// 		float64[] cov_3d
		std_msgs::Float64MultiArray arr2;
		tf::matrixEigenToMsg(link.cov_3d, arr2);
		m.cov_3d = arr2;
		m.score = link.score;
	}
	
	void fuserHMTToMsg(const NDTFeatureFuserHMT& fuser, ndt_feature::NDTFeatureFuserHMTMsg& m){
		tf::poseEigenToMsg (fuser.Tnow, m.Tnow);
		tf::poseEigenToMsg (fuser.Tlast_fuse, m.Tlast_fuse);		
		tf::poseEigenToMsg (fuser.Todom, m.Todom);
		ndt_map::NDTMapMsg mapmsg;
		bool good = lslgeneric::toMessage(fuser.map, m.map, "/world");
		m.ctr = fuser.ctr;
	}
	
	void nodeToMsg(const NDTFeatureNode& node, ndt_feature::NDTNodeMsg& m){
		fuserHMTToMsg(node.getFuser(), m.map);
		tf::poseEigenToMsg (node.T, m.T);
		tf::matrixEigenToMsg(node.cov, m.cov);
		tf::poseEigenToMsg (node.Tlocal_odom, m.Tlocal_odom);
		tf::poseEigenToMsg (node.Tlocal_fuse, m.Tlocal_fuse);
		m.nbUpdates = node.nbUpdates;
	}
	
	
	void NDTGraphToMsg(const NDTFeatureGraph& graph, ndt_feature::NDTGraphMsg& m){
		for(size_t i = 0 ; i < graph.getNbNodes() ; ++i){
			ndt_feature::NDTNodeMsg nodemsg;
			nodeToMsg(graph.getNode(i), nodemsg);
			m.nodes.push_back(nodemsg);
		}
		for(size_t i = 0 ; i < graph.getNbLinks() ; ++i){
			ndt_feature::NDTEdgeMsg edgemsg;
			edgeToMsg(graph.getLink(i), edgemsg);
			m.edges.push_back(edgemsg);
		}
		
		tf::poseEigenToMsg (graph.sensor_pose_, m.sensor_pose_);
		tf::poseEigenToMsg (graph.Tnow, m.Tnow);
		m.distance_moved_in_last_node_ = graph.getDistanceTravelled();
	}
	
}

#endif