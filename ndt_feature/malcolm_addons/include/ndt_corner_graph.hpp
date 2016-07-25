#ifndef NDTFEATURE_NDTGRAPHCORNER_24072016
#define NDTFEATURE_NDTGRAPHCORNER_24072016

#include "ndt_feature/ndt_feature_graph.h"
#include "ndt_corner_node.hpp"

#include "das/NDTCorner.hpp"


namespace ndt_feature{
	
	//It would make more sense to have a node class to I could derive from or to use a templated graph so that I can just inherite from Node and use it in the _ndt_graph instead of that vector trick.
	class NDTFeatureGraphCorner : public NDTFeatureGraph{
		
	protected:
		
		std::deque<AASS::das::NDTCorner> _ndt_corner_vector;
		std::deque<NDTCornerNode> _corner_nodes;
		
	public:
		
	NDTFeatureGraphCorner(){};
	NDTFeatureGraphCorner(const NDTFeatureGraph::Params &params, const NDTFeatureFuserHMT::Params &fuserParams) : NDTFeatureGraph(params, fuserParams){};
	
	NDTCornerNode& getCornerNode(size_t idx) {
		return _corner_nodes[idx];
	}

	const NDTCornerNode& getCornerNode(size_t idx) const {
		return _corner_nodes[idx];
	}
	
	size_t getNbLandmarks() const {return _corner_nodes.size();}
	
	void extractCorners(){			
		
		//For every node extract corners
		for (size_t i = 0; i < this->getNbNodes(); ++i) {
			std::cout << "Adding a map to extract corner " << i << std::endl;
			NDTFeatureNode* feature = new NDTFeatureNode();
			feature->copyNDTFeatureNode( (const NDTFeatureNode&)this->getNodeInterface(i) );
			
			lslgeneric::NDTMap ndt_map = feature->getNDTMap();			
			
			AASS::das::NDTCorner corner_extract;
			std::cout << "Getting all the corners" << std::endl;
			auto cells = corner_extract.getAllCorners(ndt_map);
			std::cout << "GOt : " << corner_extract.getAccurateCorners().size() << " corners " << std::endl;
			_ndt_corner_vector.push_front(corner_extract);
		}
		
		//Create all landmark with list of nodes that see them
		
		//i represent the node from where the corner as been seen
		for(size_t i = 0 ; i < _ndt_corner_vector.size() ; ++i){
			
			std::vector< Eigen::Vector3d > corners = _ndt_corner_vector[i].getAccurateCorners();
			for(size_t j = 0 ; j < corners.size() ; ++j){
				int index = seenCorner(corners[j]);
				//Not seen before so new corner
				if( index == -1){
					std::cout << "Adding new corner" << std::endl;
					NDTCornerNode corner_node;
					corner_node.setPose(corners[j]);
					corner_node.seenBy(i);
					_corner_nodes.push_back(corner_node);
				}
				//Seen before so we just add a new vision from a nodez
				else{
					std::cout << "Corner already exist" << std::endl;
					_corner_nodes[index].seenBy(i);
				}
				
			}
			
			
		}
		//From all result, update the graph by adding the landmarks

	}
	
	int seenCorner(const Eigen::Vector3d& cor) const{
	
		for(size_t i = 0 ; i < _corner_nodes.size() ; ++i){
			if(cor == _corner_nodes[i].getPose()){
				return i;
			}
		}
		return -1;
	}
	
	
	/**
	 * @brief Return link between node and landmark. The reference is the node
	 */ 
	std::vector<NDTFeatureLink> getLandmarkLinks() const {
		
		std::vector<NDTFeatureLink> links;
		//i represent the number of the corner and j is the number of the ode seeing the corner
		for(size_t i = 0 ; i < _corner_nodes.size() ; ++i){
			std::vector<int> seenby = _corner_nodes[i].getSeen();
			for(size_t j = 0 ; j < seenby.size() ; ++j){
				NDTFeatureLink m(seenby[j], getNbNodes() + i);
				
				//TODO add transforms
				
				
				links.push_back(m);
				
			}
		}
		return links;
		
	}
	

		
	};
	
}

#endif