#include "G2OGraphMaker.hpp"
#include "G2OGraphMakerNormal.hpp"


int main(){
	
	ndt_feature::G2OGraphMarker g2o_graph(g2o::SE2(0.2, 0.1, -0.1), 
		Eigen::Vector2d(0.05, 0.01), 
		DEG2RAD(2.),
		Eigen::Vector2d(0.05, 0.05),
		Eigen::Vector2d(0.05, 0.01),
		Eigen::Vector2d(0.2, 0.2));
	
	
	std::ifstream in("robotposes.g2o");
	g2o_graph.addRobotPoseAndOdometry(in);
	
	std::ifstream in2("landmarks.g2o");
	g2o_graph.addLandmarkAndObservation(in2);
	
	g2o_graph.makeGraph();
	
	g2o_graph.save("final.g2o");
	
}