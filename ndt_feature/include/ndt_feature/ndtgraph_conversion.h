#ifndef NDT_FEATURE_CONVERSIONS_NDT_GRAPH_22102016

#define NDT_FEATURE_CONVERSIONS_NDT_GRAPH_22102016

#include "ndt_feature_graph.h"
#include "ndt_feature/NDTEdgeMsg.h"
#include "ndt_feature/NDTNodeMsg.h"
#include "ndt_feature/NDTGraphMsg.h"
#include "ndt_feature/NDTFeatureFuserHMTMsg.h"
#include "eigen_conversions/eigen_msg.h"


namespace ndt_feature {
	
	
	
	inline void edgeToMsg(const NDTFeatureLink& link, ndt_feature::NDTEdgeMsg& m){
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
	
	inline void fuserHMTToMsg(const NDTFeatureFuserHMT& fuser, ndt_feature::NDTFeatureFuserHMTMsg& m){
		tf::poseEigenToMsg (fuser.Tnow, m.Tnow);
		tf::poseEigenToMsg (fuser.Tlast_fuse, m.Tlast_fuse);		
		tf::poseEigenToMsg (fuser.Todom, m.Todom);
		
// 		std::cout << "Nb of init when creating message (more than 0) " << fuser.map->getAllInitializedCells().size() << std::endl;
		bool good = lslgeneric::toMessage(fuser.map, m.map, "/world");
		m.ctr = fuser.ctr;
		assert(m.map.header.frame_id != "");
	}
	
	inline void nodeToMsg(const NDTFeatureNode& node, ndt_feature::NDTNodeMsg& m){
		
// 		std::cout << "GETTING THE NODE FUCKER TO WORK gosh" << node.map->Tnow.matrix() << std::endl;
		
		fuserHMTToMsg(node.getFuser(), m.map);
		tf::poseEigenToMsg (node.T, m.T);
		tf::matrixEigenToMsg(node.cov, m.cov);
		tf::poseEigenToMsg (node.Tlocal_odom, m.Tlocal_odom);
		tf::poseEigenToMsg (node.Tlocal_fuse, m.Tlocal_fuse);
		m.nbUpdates = node.nbUpdates;
		m.time_last_update = node.time_last_update;
	}
	
	
	inline void NDTGraphToMsg(const NDTFeatureGraph& graph, ndt_feature::NDTGraphMsg& m, const std::string& frame){
		
		m.header.stamp=ros::Time::now();
		m.header.frame_id = frame;//is it in *map? 
		
		std::cout << "Nodes \n";
		for(size_t i = 0 ; i < graph.getNbNodes() ; ++i){
			ndt_feature::NDTNodeMsg nodemsg;
			nodeToMsg(graph.getNode(i), nodemsg);
			m.nodes.push_back(nodemsg);
		}
		std::cout << "Edge \n";
		for(size_t i = 0 ; i < graph.getNbLinks() ; ++i){
			ndt_feature::NDTEdgeMsg edgemsg;
			edgeToMsg(graph.getLink(i), edgemsg);
			m.edges.push_back(edgemsg);
		}
		
		std::cout << "Poses \n";
		tf::poseEigenToMsg (graph.sensor_pose_, m.sensor_pose_);
		tf::poseEigenToMsg (graph.Tnow, m.Tnow);
		m.distance_moved_in_last_node_ = graph.getDistanceTravelled();
		
		std::cout << "ewnd frame " << m.header.frame_id << std::endl;
		assert(m.header.frame_id != "");
	}
	
	
	
	
	inline void msgToEdge(const ndt_feature::NDTEdgeMsg& m, NDTFeatureLink& link){
		link.ref_idx = m.ref_idx;
		link.mov_idx = m.mov_idx;
		tf::poseMsgToEigen (m.T, link.T);
		std::vector<double>::const_iterator it;
		it=m.cov.data.begin();
		
// 		std::cout << "Cov size " << m.cov.data.size() << std::endl;
		assert(m.cov.data.size() == 9);
		
// 		for(it ; it != m.cov.data.end() ; it++){
// 			std::cout <<"Got " << *it << "\n" ;
// 		}
		
		it=m.cov.data.begin();
		link.cov <<   *it, *(it+1), *(it+2), 
					*(it+3), *(it+4), *(it+5), 
					*(it+6), *(it+7), *(it+8);
// 		float64[] cov_3d
					
		if(m.cov_3d.data.size() > 0){
			
			std::vector<double>::const_iterator it_2;
			it_2=m.cov_3d.data.begin();
			std::cout << m.cov_3d.data.size() << std::endl;
			assert(m.cov_3d.data.size() == 36);
			
			Eigen::MatrixXd cov(6,6);
			for(size_t i = 0; i < 6 ; ++i){
				for(size_t j = 0 ; j < 6 ; ++j){
					cov(i, j) = m.cov_3d.data[ (6*i) + j];
				}
			}
			
			link.cov_3d = cov;
		}
					
		link.score = m.score;
	}
	
	inline void msgTofuserHMT(const ndt_feature::NDTFeatureFuserHMTMsg& m, NDTFeatureFuserHMT& fuser, std::string& frame){
// 		std::cout << "What the heck ? " << std::endl;
// 		std::cout << "Doing the TNOW" << fuser.Tnow.matrix() <<std::endl;
		tf::poseMsgToEigen (m.Tnow, fuser.Tnow);
// 		std::cout << "Doing the TLASTFUSE" << fuser.Tlast_fuse.matrix() << std::endl;
		tf::poseMsgToEigen (m.Tlast_fuse, fuser.Tlast_fuse);	
// 		std::cout << "Doing TODOM" << fuser.Todom.matrix() << std::endl;	
		tf::poseMsgToEigen (m.Todom, fuser.Todom);
		ndt_map::NDTMapMsg mapmsg;
		
// 		std::cout << "Doing the NDT map" << std::endl;
		//COMMENTED THIS LINES
// 		lslgeneric::NDTMap* map = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(fuser.params_.resolution));
		
		if(fuser.map != NULL){
// 			std::cout << "DELETING THE FUSER MAP" << std::endl;
			delete fuser.map;
		}
		else{
// 			std::cout << "No deleting the fuser's map" << std::endl;
		}
		
		lslgeneric::LazyGrid* lz;
		bool good = lslgeneric::fromMessage(lz, fuser.map, m.map, frame, true);
		
		
// 		std::cout << "Nb of init (Should be more) " << fuser.map->getAllInitializedCells().size() << std::endl;
		
// 		std::cout << "ctr" << std::endl;
		fuser.ctr = m.ctr;
	}
	
	inline void msgToNode(const ndt_feature::NDTNodeMsg& m, NDTFeatureNode& node, std::string& frame){
		
// 		std::cout << "HMT convertion" << std::endl;
		msgTofuserHMT(m.map, node.getFuser(), frame);
		
// 		std::cout << "All frames" << std::endl;
		tf::poseMsgToEigen (m.T, node.T);
		tf::poseMsgToEigen (m.Tlocal_odom, node.Tlocal_odom);
		tf::poseMsgToEigen (m.Tlocal_fuse, node.Tlocal_fuse);
		
// 		std::cout << "Updates" << std::endl;
		node.nbUpdates = m.nbUpdates;
		node.time_last_update = m.time_last_update;
		
		std::vector<double>::const_iterator it;
		it=m.cov.data.begin();
		Eigen::Matrix3d cov;
		for(size_t i = 0; i < 3 ; ++i){
			for(size_t j = 0 ; j < 3 ; ++j){
// 				std::cout << "Copying data" << *it << std::endl;
				cov(i, j) = *it;
				++it;
			}
		}
		node.cov = cov;
		
	}
	
	inline void msgToNDTGraph(const ndt_feature::NDTGraphMsg& m, NDTFeatureGraph& graph, std::string& frame){
// 		std::cout << "Nodes " << std::endl;
		for(size_t i = 0 ; i < m.nodes.size() ; ++i){
			ndt_feature::NDTFeatureNode node;
			
			//ATTENTION NOT TRUE: MEMORY LEAK FOR NOW
			node.map = new ndt_feature::NDTFeatureFuserHMT( ndt_feature::NDTFeatureFuserHMT::Params() );
			
			msgToNode(m.nodes[i], node, frame);
			graph.push_back(node);
		}
		for(size_t i = 0 ; i < m.edges.size() ; ++i){
			ndt_feature::NDTFeatureLink link;
			msgToEdge(m.edges[i], link);
			graph.push_back(link);
		}
// 		std::cout << "Edge " << std::endl;
// 		for(size_t i = 0 ; i < graph.getNbLinks() ; ++i){
// 			ndt_feature::NDTEdgeMsg edgemsg;
// 			edgeToMsg(graph.getLink(i), edgemsg);
// 			m.edges.push_back(edgemsg);
// 		}
// 		
// 		std::cout << "Poses" << std::endl;
		tf::poseMsgToEigen (m.sensor_pose_, graph.sensor_pose_);
		tf::poseMsgToEigen (m.Tnow, graph.Tnow);
		graph.setDistanceTravelled(m.distance_moved_in_last_node_);
	}
	
}

#endif