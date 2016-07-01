#ifndef NDTFEATURE_NDTREGISTRATIONGRAPH_27062016
#define NDTFEATURE_NDTREGISTRATIONGRAPH_27062016

#include <bettergraph/DirectedPseudoGraph.hpp>

#include "ndt_feature/ndt_feature_graph.h"

namespace ndt_feature {
	
	class NDTFeatureRegistrationGraph : public bettergraph::DirectedPseudoGraph<ndt_feature::NDTFeatureNode, NDTFeatureLink>{
		
	protected:
		
		
	public:
		
		typedef typename bettergraph::DirectedPseudoGraph<ndt_feature::NDTFeatureNode, NDTFeatureLink>::GraphType NDTFeatureRegistrationGraphType;
		typedef typename bettergraph::DirectedPseudoGraph<ndt_feature::NDTFeatureNode, NDTFeatureLink>::Vertex VertexNDTFeatureRegistrationGraph;
		typedef typename bettergraph::DirectedPseudoGraph<ndt_feature::NDTFeatureNode, NDTFeatureLink>::Edge EdgeNDTFeatureRegistrationGraph;
		typedef typename bettergraph::DirectedPseudoGraph<ndt_feature::NDTFeatureNode, NDTFeatureLink>::VertexIterator VertexIteratorNDTFeatureRegistrationGraph;
		typedef typename bettergraph::DirectedPseudoGraph<ndt_feature::NDTFeatureNode, NDTFeatureLink>::EdgeIterator EdgeIteratorNDTFeatureRegistrationGraph;
		NDTFeatureRegistrationGraph(){};
		
		void convert(const ndt_feature::NDTFeatureGraph& ndt_graph);		
	};
	
	
	inline void NDTFeatureRegistrationGraph::convert(const ndt_feature::NDTFeatureGraph& ndt_graph)
	{
		this->clear();
		size_t si = ndt_graph.getNbNodes();
		std::deque<VertexNDTFeatureRegistrationGraph> vertex_deque;
		//Adding all nodes
		for(size_t i = 0 ; i <si ; ++i){
			VertexNDTFeatureRegistrationGraph v;
			NDTFeatureNode feature;
			feature.copyNDTFeatureNode( (const NDTFeatureNode&)ndt_graph.getNodeInterface(i) );
			addVertex(v, feature);
			vertex_deque.push_back(v);
		}
		
// 		Adding all the edges
		size_t si_edge = ndt_graph.getNbLinks();
		for(size_t i = 0 ; i <si_edge ; ++i){
			VertexNDTFeatureRegistrationGraph v;
			NDTFeatureLink link = NDTFeatureLink((const NDTFeatureLink&) ndt_graph.getLinkInterface(i));
			size_t origin = link.getRefIdx();
			size_t destination = link.getMovIdx();
			EdgeNDTFeatureRegistrationGraph ed;
			addEdge(ed, vertex_deque[origin], vertex_deque[destination], link);
		}
		
		assert(this->getNumVertices() == ndt_graph.getNbNodes() || "Number of node is not the same");
		assert(this->getNumEdges() == ndt_graph.getNbLinks() || "Number of node is not the same");
		
	}
	
	
	

		
		
}


#endif