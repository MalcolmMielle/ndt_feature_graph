#include "das/RANSACAndCPD.hpp"
#include "das/RANSAC.hpp"
#include "das/priors/BasementPriorLine.hpp"
#include "G2OGraphMaker.hpp"
#include "vodigrex/utils/Utils.hpp"

#include "G2OGraphMaker.hpp"
#include "G2OGraphMakerNormal.hpp"


int main(){
	
	ndt_feature::G2OGraphMarker g2o_graph(g2o::SE2(0.2, 0.1, -0.1), 
		Eigen::Vector2d(0.05, 0.01), 
		DEG2RAD(2.),
		Eigen::Vector2d(0.05, 0.05),
		Eigen::Vector2d(0.05, 0.01),
		Eigen::Vector2d(0.2, 0.2));
	
	/************* ADD ROBOT POSES AND LANDMARKS*********/
	
	std::ifstream in("robotposes.g2o");
	g2o_graph.addRobotPoseAndOdometry(in);
	
	std::ifstream in2("landmarks.g2o");
	g2o_graph.addLandmarkAndObservation(in2);
	
	
	/************ ADD PRIOR**************/
	
	AASS::das::BasementPriorLine basement;
	cv::Mat src = cv::imread( basement.map, CV_LOAD_IMAGE_COLOR ), src_gray;
	cv::cvtColor(src, src_gray, CV_RGB2GRAY );
	
	cv::imshow("src", src);
	cv::imshow("src gray", src_gray);
	cv::threshold(src_gray, src_gray, 100, 255, src_gray.type());
// 	src_gray.convertTo(src_gray, CV_32FC1);
	
	cv::imshow("src gray convert", src_gray);
	cv::waitKey(0);
  
	AASS::das::CornerDetector cornerDetect;
	
	cornerDetect.getFeaturesGraph(basement);
	cornerDetect.removeClosePoints(20);
	auto corners_prior = cornerDetect.getGraphPoint();
	
	std::ifstream in_scale("association.txt");
	double scale;
	in_scale >> scale;
	scale = 1 / scale;
	g2o_graph.setScalePriorToLandmarks(scale);
	g2o_graph.addAllPriors(cornerDetect.getGraph());
	g2o_graph.scalePrior();
	
	/************** ADD LINKS BETWEEN THE MAPS ***********/
		
	std::ifstream in3("association.txt");
	g2o_graph.addLinkBetweenMaps(in3);
	
	/************** MAKE GRAPH***********/
	
	g2o_graph.makeGraph();
	
	g2o_graph.save("final.g2o");
	
}