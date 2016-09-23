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
		std::cout << "Nodes " << std::endl;
		for(size_t i = 0 ; i < graph.getNbNodes() ; ++i){
			ndt_feature::NDTNodeMsg nodemsg;
			nodeToMsg(graph.getNode(i), nodemsg);
			m.nodes.push_back(nodemsg);
		}
		std::cout << "Edge " << std::endl;
		for(size_t i = 0 ; i < graph.getNbLinks() ; ++i){
			ndt_feature::NDTEdgeMsg edgemsg;
			edgeToMsg(graph.getLink(i), edgemsg);
			m.edges.push_back(edgemsg);
		}
		
		std::cout << "Poses" << std::endl;
		tf::poseEigenToMsg (graph.sensor_pose_, m.sensor_pose_);
		tf::poseEigenToMsg (graph.Tnow, m.Tnow);
		m.distance_moved_in_last_node_ = graph.getDistanceTravelled();
	}
	
	
	
	
	void msgToEdge(const ndt_feature::NDTEdgeMsg& m, NDTFeatureLink& link){
		link.ref_idx = m.ref_idx;
		link.mov_idx = m.mov_idx;
		tf::poseMsgToEigen (m.T, link.T);
		std::vector<double>::const_iterator it;
		it=m.cov.data.begin();
		
		std::cout << "Cov size " << m.cov.data.size() << std::endl;
		assert(m.cov.data.size() == 9);
		
		for(it ; it != m.cov.data.end() ; it++){
			std::cout <<"Got " << *it << "\n" ;
		}
		
		it=m.cov.data.begin();
		link.cov <<   *it, *(it+1), *(it+2), 
					*(it+3), *(it+4), *(it+5), 
					*(it+6), *(it+7), *(it+8);
// 		float64[] cov_3d
		std::vector<double>::const_iterator it_2;
		it_2=m.cov_3d.data.begin();
		assert(m.cov_3d.data.size() == 9);
		link.cov_3d << *it_2, *(it_2+1), *(it_2+2), 
					*(it_2+3), *(it_2+4), *(it_2+5), 
					*(it_2+6), *(it_2+7), *(it_2+8);
					
		link.score = m.score;
	}
	
	void msgTofuserHMT(const ndt_feature::NDTFeatureFuserHMTMsg& m, NDTFeatureFuserHMT& fuser, std::string& frame){
		tf::poseMsgToEigen (m.Tnow, fuser.Tnow);
		tf::poseMsgToEigen (m.Tlast_fuse, fuser.Tlast_fuse);		
		tf::poseMsgToEigen (m.Todom, fuser.Todom);
		ndt_map::NDTMapMsg mapmsg;
		
		lslgeneric::LazyGrid *lz = dynamic_cast<lslgeneric::LazyGrid*>(fuser.map->getMyIndex() );
		bool good = lslgeneric::fromMessage(lz, fuser.map, m.map, frame);
		fuser.ctr = m.ctr;
	}
	
	void msgToNode(){
		
	}
	
	void msgToNDTGraph(){
		
	}
	
}

#endif