// #include "das/RANSACAndCPD.hpp"
// #include "das/RANSAC.hpp"
// #include "das/priors/BasementPriorLine.hpp"
// #include "G2OGraphMaker.hpp"
// #include "vodigrex/utils/Utils.hpp"

#include "G2OGraphMaker.hpp"
// #include "G2OGraphMakerNormal.hpp"


int main(){
	
	ndt_feature::G2OGraphMarker g2o_graph(g2o::SE2(0.2, 0.1, -0.1), //sensor offset
		Eigen::Vector2d(0.0005, 0.0001), //Robot translation noise
		DEG2RAD(2.), 				//Rotation noise for robot
		Eigen::Vector2d(0.0005, 0.0005), //Landmarks noise
		Eigen::Vector2d(1, 0.001), //Prior noise
		Eigen::Vector2d(0.2, 0.2)); //Link noise
	
	/************* ADD LANDMARKS*********/
	Eigen::Vector3d robot0;
	robot0 << 0, 0, 0;
	g2o_graph.addRobotPose(robot0);
	Eigen::Vector3d robot1;
	robot1 << 1, 0, 0;
	g2o_graph.addRobotPose(robot1);
	Eigen::Vector3d robot2;
	robot2 << 1, 1, 0;
	g2o_graph.addRobotPose(robot2);
	Eigen::Vector3d robot3;
	robot3 << 2, 1, 0;
	g2o_graph.addRobotPose(robot3);
	Eigen::Vector3d robot4;
	robot4 << 2, 3, 0;
	g2o_graph.addRobotPose(robot4);
	Eigen::Vector3d robot5;
	robot5 << 1, 3, 0;
	g2o_graph.addRobotPose(robot5);
	Eigen::Vector3d robot6;
	robot6 << 1, 2, 0;
	g2o_graph.addRobotPose(robot6);
	Eigen::Vector3d robot7;
	robot7 << 0, 2, 0;
	g2o_graph.addRobotPose(robot7);
	
	g2o::SE2 od1(1, 0, 0);
	g2o_graph.addOdometry(od1, 0, 1);
	g2o::SE2 od2(0, 1, 0);
	g2o_graph.addOdometry(od2, 1, 2);
	g2o::SE2 od3(1, 0, 0);
	g2o_graph.addOdometry(od3, 2, 3);
	g2o::SE2 od4(0, 2, 0);
	g2o_graph.addOdometry(od4, 3, 4);
	g2o::SE2 od5(-1, 0, 0);
	g2o_graph.addOdometry(od5, 4, 5);
	g2o::SE2 od6(0, -1, 0);
	g2o_graph.addOdometry(od6, 5, 6);
	g2o::SE2 od7(-1, 0, 0);
	g2o_graph.addOdometry(od7, 6, 7);
	g2o::SE2 od8(0, -2, 0);
	g2o_graph.addOdometry(od8, 7, 8);
	
	
	/********* ADD Prior ************/
	Eigen::Vector3d robot9;
	robot9 << -1, -1, 0;
	Eigen::Vector3d robot10;
	robot10 << 2, -1, 0;
	Eigen::Vector3d robot11;
	robot11 << 2, 0, 0;
	Eigen::Vector3d robot12;
	robot12 << 3, 0, 0;
	Eigen::Vector3d robot13;
	robot13 << 3, 4, 0;
	Eigen::Vector3d robot14;
	robot14 << 0, 4, 0;
	Eigen::Vector3d robot15;
	robot15 << 0, 3, 0;
	Eigen::Vector3d robot16;
	robot16 << -1, 3, 0;
	
	g2o_graph.addPriorLandmarkPose(robot9);
	g2o_graph.addPriorLandmarkPose(robot10);
	g2o_graph.addPriorLandmarkPose(robot11);
	g2o_graph.addPriorLandmarkPose(robot12);
	g2o_graph.addPriorLandmarkPose(robot13);
	g2o_graph.addPriorLandmarkPose(robot14);
	g2o_graph.addPriorLandmarkPose(robot15);
	g2o_graph.addPriorLandmarkPose(robot16);
	
	g2o::SE2 od9(3, 0, 0);
	g2o::SE2 od10(0, 1, 0);
	g2o::SE2 od11(1, 0, 0);
	g2o::SE2 od12(0, 4, 0);
	g2o::SE2 od13(-3, 0, 0);
	g2o::SE2 od14(0, -1, 0);
	g2o::SE2 od15(-2, 0, 0);
	g2o::SE2 od16(0, -4, 0);
	
	// -1 in the numbers of the prior
	g2o_graph.addEdgePrior(od9, 8, 9);
	g2o_graph.addEdgePrior(od10, 9, 10);
	g2o_graph.addEdgePrior(od11, 10, 11);
	g2o_graph.addEdgePrior(od12, 11, 12);
	g2o_graph.addEdgePrior(od12, 12, 13);
	g2o_graph.addEdgePrior(od13, 13, 14);
	g2o_graph.addEdgePrior(od14, 14, 15);
	g2o_graph.addEdgePrior(od16, 15, 8);
	
// 	Eigen::Vector3d prior_lan4;
// 	prior_lan4 << 0, 1, 0;
// 	g2o_graph.addPriorLandmarkPose(prior_lan4);
// 	Eigen::Vector3d prior_lan5;
// 	prior_lan5 << 7, 1, 0;
// 	g2o_graph.addPriorLandmarkPose(prior_lan5);
// 	Eigen::Vector3d prior_lan6;
// 	prior_lan6 << 10, 1, 0;
// 	g2o_graph.addPriorLandmarkPose(prior_lan6);
// 	
// 	g2o::SE2 edge2(7, 1, 0);
// 	std::tuple<g2o::SE2, int, int> ed2(edge2, 4, 5);
// 	g2o_graph.addEdgePrior(ed2);
// 	g2o::SE2 edge3(3, 0, 0);
// 	std::tuple<g2o::SE2, int, int> ed3(edge3, 5, 6);
// 	g2o_graph.addEdgePrior(ed3);
// 	
// 	/********* ADD Links*******/
// 	
	g2o::SE2 link1(0, 0, 0);
	std::tuple<g2o::SE2, int, int> lnk1(link1, 0, 8);
	g2o_graph.addLinkBetweenMaps(lnk1);
	g2o::SE2 link2(0, 0, 0);
	std::tuple<g2o::SE2, int, int> lnk2(link2, 1, 9);
	g2o_graph.addLinkBetweenMaps(lnk2);
	g2o::SE2 link3(0, 0, 0);
	std::tuple<g2o::SE2, int, int> lnk3(link3, 2, 10);
	g2o_graph.addLinkBetweenMaps(lnk3);
	std::tuple<g2o::SE2, int, int> lnk4(link3, 3, 11);
	g2o_graph.addLinkBetweenMaps(lnk4);
	std::tuple<g2o::SE2, int, int> lnk5(link3, 4, 12);
	g2o_graph.addLinkBetweenMaps(lnk5);
	std::tuple<g2o::SE2, int, int> lnk6(link3, 5, 13);
	g2o_graph.addLinkBetweenMaps(lnk6);
	std::tuple<g2o::SE2, int, int> lnk7(link3, 6, 14);
	g2o_graph.addLinkBetweenMaps(lnk7);
	std::tuple<g2o::SE2, int, int> lnk8(link3, 7, 15);
	g2o_graph.addLinkBetweenMaps(lnk8);
	
	g2o_graph.makeGraph();
	
	g2o_graph.save("cov_test.g2o");
	
	
}