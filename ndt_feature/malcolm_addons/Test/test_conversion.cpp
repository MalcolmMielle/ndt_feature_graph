
#include "ndt_feature/ndtgraph_conversion.h"

int main(){
	
	ndt_feature::NDTEdgeMsg edgemsg;
	ndt_feature::NDTFeatureLink edge;
	
	std::cout << "NODE to msg" << std::endl;
	ndt_feature::edgeToMsg(edge, edgemsg);
	
	
	std::cout << "MSG to node" << std::endl;
	ndt_feature::NDTFeatureLink edge2;
	ndt_feature::msgToEdge(edgemsg, edge2);
	
	std::cout << "Done" << std::endl;
	
	assert(edge.cov == edge2.cov);
	assert(edge.cov_3d == edge2.cov_3d);
	
	ndt_feature::NDTNodeMsg nodemsg;
	ndt_feature::NDTFeatureNode node;
	
	node.Tlocal_odom.matrix() << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
	
	std::string frame;
	std::cout << "NODE to msg" << std::endl;
	ndt_feature::nodeToMsg(node, nodemsg);
	
	
	std::cout << "MSG to node" << std::endl;
	ndt_feature::NDTFeatureNode node2;
	ndt_feature::msgToNode(nodemsg, node2, frame);
	
	std::cout << "Done" << std::endl;
	
	assert(node.cov == node2.cov);
	assert(node.getFuser().getCov() == node.getFuser().getCov() );
	assert(node.getTLocalOdom().matrix() == node2.getTLocalOdom().matrix());
}