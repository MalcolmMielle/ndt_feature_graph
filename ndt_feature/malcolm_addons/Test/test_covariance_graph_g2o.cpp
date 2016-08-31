#include "das/RANSACAndCPD.hpp"
#include "das/RANSAC.hpp"
#include "das/priors/BasementPriorLine.hpp"
#include "G2OGraphMaker.hpp"
#include "vodigrex/utils/Utils.hpp"

#include "G2OGraphMaker.hpp"
#include "G2OGraphMakerNormal.hpp"


int main(){
	
	ndt_feature::G2OGraphMarker g2o_graph(g2o::SE2(0.2, 0.1, -0.1), //sensor offset
		Eigen::Vector2d(0.05, 0.01), //Robot translation noise
		DEG2RAD(2.), 				//Rotation noise for robot
		Eigen::Vector2d(0.05, 0.05), //Landmarks noise
		Eigen::Vector2d(0, 0), //Prior noise
		Eigen::Vector2d(0.2, 0.2)); //Link noise
	
	/************* ADD LANDMARKS*********/
	Eigen::Vector3d robot0;
	robot0 << 0, 0, 0;
	g2o_graph.addRobotPose(robot0);
	Eigen::Vector3d robot1;
	robot1 << 1, 0, 0;
	g2o_graph.addRobotPose(robot1);
	Eigen::Vector3d robot2;
	robot2 << 1, -10, 0;
	g2o_graph.addRobotPose(robot2);
	Eigen::Vector3d robot3;
	robot3 << 0, -10, 0;
	g2o_graph.addRobotPose(robot3);
	
	g2o::SE2 od1(1, 0, 0);
	g2o_graph.addOdometry(od1, 0, 1);
	g2o::SE2 od2(0, -10, 0);
	g2o_graph.addOdometry(od2, 1, 2);
	g2o::SE2 od3(-1, 0, 0);
	g2o_graph.addOdometry(od3, 2, 3);
	
	
	/********* ADD Prior ************/
	Eigen::Vector3d prior_lan4;
	prior_lan4 << 0, 1, 0;
	g2o_graph.addPriorLandmarkPose(prior_lan4);
	Eigen::Vector3d prior_lan5;
	prior_lan5 << 7, 1, 0;
	g2o_graph.addPriorLandmarkPose(prior_lan5);
	Eigen::Vector3d prior_lan6;
	prior_lan6 << 10, 1, 0;
	g2o_graph.addPriorLandmarkPose(prior_lan6);
	
	g2o::SE2 edge2(7, 1, 0);
	std::tuple<g2o::SE2, int, int> ed2(edge2, 4, 5);
	g2o_graph.addEdgePrior(ed2);
	g2o::SE2 edge3(3, 0, 0);
	std::tuple<g2o::SE2, int, int> ed3(edge3, 5, 6);
	g2o_graph.addEdgePrior(ed3);
	
	/********* ADD Links*******/
	
	g2o::SE2 link1(0, 0, 0);
	std::tuple<g2o::SE2, int, int> lnk1(link1, 0, 4);
	g2o_graph.addLinkBetweenMaps(lnk1);
	g2o::SE2 link2(0, 0, 0);
	std::tuple<g2o::SE2, int, int> lnk2(link2, 1, 5);
	g2o_graph.addLinkBetweenMaps(lnk2);
	g2o::SE2 link3(0, 0, 0);
	std::tuple<g2o::SE2, int, int> lnk3(link3, 2, 6);
	g2o_graph.addLinkBetweenMaps(lnk3);
	
	g2o_graph.makeGraph();
	
	g2o_graph.save("cov_test.g2o");
	
	
}