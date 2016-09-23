
#include "ndt_feature/ndtgraph_conversion.h"

int main(){
	
	ndt_feature::NDTNodeMsg nodemsg;
	ndt_feature::NDTFeatureNode node;
	
	std::cout << "NODE to msg" << std::endl;
	ndt_feature::nodeToMsg(node, nodemsg);
	
	
	std::cout << "MSG to node" << std::endl;
	ndt_feature::NDTFeatureNode node2;
	ndt_feature::msgToNode(nodemsg, node2);
	
	std::cout << "Done" << std::endl;
	
	assert(node == node2);
	
	
}