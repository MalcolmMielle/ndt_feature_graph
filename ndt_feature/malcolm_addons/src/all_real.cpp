#include "das/RANSACAssociation.hpp"
#include "das/RANSAC.hpp"
#include "das/priors/BasementPriorLine.hpp"
#include "das/Utils.hpp"
#include "das/NDTCorner.hpp"
#include "das/conversion.hpp"
#include "vodigrex/utils/Utils.hpp"

#include <ros/ros.h>

#include <ndt_map/NDTMapMsg.h>
#include <ndt_map/ndt_conversions.h>

// #include "G2OGraphMaker.hpp"
// #include "conversion.hpp"
// #include "ndt_feature/ndt_feature_graph.h"

cv::Mat rotate(cv::Mat src, double angle)
{
    cv::Mat dst;
    cv::Point2f pt(src.cols/2., src.rows/2.);    
    cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);
    cv::warpAffine(src, dst, r, cv::Size(src.cols, src.rows));
    return dst;
}


int main( int argc, char** argv )
{
	// 	//*****************************First load the map and calculate points
	std::string file = "/home/malcolm/Documents/basement2d_map.jff";
// 	map.loadFromJFF(file.c_str());
	double resolution = 0.2;
	auto mapGrid = new lslgeneric::LazyGrid(resolution);
	lslgeneric::NDTMap map(mapGrid);
	if(map.loadFromJFF(file.c_str()) < 0)
		std::cout << "File didn't load" << std::endl;
	std::cout << "File loaded" << std::endl;
	
	auto cells = map.getAllInitializedCells();
	
	double x, y, z;
	map.getCellSizeInMeters(x, y, z);
	
	std::cout << "Size : " << x <<" " << y<< " " << z << std::endl;
	
// 	exit(0);
	
	for(size_t i = 0 ; i < cells.size() ; ++i){
// 		std::cout << "Classifying" << std::endl;
		cells[i]->classify();
		lslgeneric::NDTCell cell;
		assert(cells[i]->getClass() != cells[i]->UNKNOWN);
	}
	
// 	exit(0);
// 	
	
	/************ EXPORT NDT CORNER TO FILE ******************/

// 	AASS::das::NDTCorner corners_export;
// 	std::cout << "Searching for corners" << std::endl;
// 	auto ret_export = corners_export.getAllCorners(map);
// 	auto ret_opencv_export = corners_export.getCvCorners();
// 	std::cout << "Found " << ret_export.size() << " Corners. yes same as " << ret_opencv_export.size() << std::endl;
// 	
// 	std::ofstream out_stream("/home/malcolm/Documents/corners_ndt_accurate.txt");
// 	corners_export.exportAccurateCorners(out_stream);
// 	
// 	exit(0);
	
	/************READ CORNER NDT FROM FILE******************/
	AASS::das::NDTCorner corners;
	
	std::ifstream in_stream("/home/malcolm/Documents/corners_ndt_accurate.txt");
	corners.readAccurateCorners(in_stream);
	
// 	auto ret_opencv = corners.getCvCorners();
	auto ret_opencv = corners.getAccurateCvCorners();
	std::cout << "Size " << ret_opencv.size() << std::endl;
// 	assert(64 == ret_opencv.size());
	
	double max_x = ret_opencv[0].x, max_y = ret_opencv[0].y;
	double min_x = ret_opencv[0].x, min_y = ret_opencv[0].y;
	
	auto it = ret_opencv.begin();
	for(it ; it != ret_opencv.end() ; ++it){
		if(max_x < it->x){
			max_x = it->x;
		}
		if(max_y < it->y){
			max_y = it->y;
		}
		if(min_x > it->x){
			min_x = it->x;
		}
		if(min_y > it->y){
			min_y = it->y;
		}
	}
	
	std::cout << "Max : " << max_x << " " << max_y << " min : " << min_x  << " " << min_y << std::endl;
	
	AASS::das::BasementPriorLine basement;
	cv::Mat src = cv::imread( basement.map, CV_LOAD_IMAGE_COLOR ), src_gray;
	cv::cvtColor(src, src_gray, CV_RGB2GRAY );
// 	src_gray.convertTo(src_gray, CV_32FC1);
	
	double max = max_x;
	double min = min_x;
	double size = src.cols;
	bool flag_x = true;
	if(max < max_y){
		max = max_y;
		min = min_y;
		flag_x = false;
		size = src.rows;
	}
	
	int scale = (size - 10) / (max);
	it = ret_opencv.begin();
	for(it ; it != ret_opencv.end() ; ++it){
		it->x = it->x * (size - 10) / (max);
		it->y = it->y * (size - 10) / (max);
// 		it->x = it->x * 10;
// 		it->y = it->y * 10;
	}
	
	cv::Mat ndt_cv_src, ndt_cv_tmp;
	
	AASS::das::toCvMat(map, ndt_cv_src, scale);
	
	std::cout << AASS::das::type2str(ndt_cv_src.type()) << std::endl;;
	
	cv::imshow("Scale ndt map color", ndt_cv_src);
	cv::waitKey(0);
	
	cv::cvtColor( ndt_cv_src, ndt_cv_tmp, CV_RGB2GRAY );
// 	ndt_cv_tmp.convertTo(ndt_cv_tmp, CV_32FC1);
	
	cv::imshow("Scale ndt map", ndt_cv_tmp);
	cv::waitKey(0);
	
	cv::Mat ndt_cv;
	cv::Rect roi = cv::Rect(0, 0, src.cols, src.rows);
	ndt_cv = ndt_cv_tmp(roi);
	ndt_cv.convertTo(ndt_cv, CV_8U);
	cv::imshow("ndt cv used ", ndt_cv);
	//Map point to fit the size of prior image
	
	
	
	//******************** Now extract from prior
	
	AASS::das::CornerDetector cornerDetect;
	
	auto corners_prior = cornerDetect.getFeaturesGraph(basement);
	auto graph = cornerDetect.getGraph();
	
	//******************** print
	
	cv::Mat print = cv::Mat::zeros(ndt_cv.size(), 1), print_prior = cv::Mat::zeros(src_gray.size(), 1);

	for( int j = 0; j < ret_opencv.size(); j++ )
	{
		cv::circle( print, ret_opencv[j], 5,  cv::Scalar(150), 2, 8, 0 );
	// 	std::cout << ret_opencv[j] << std::endl;
	// 	cv::imshow("corners", print);
	// 	cv::waitKey(0);
	}
	for( int j = 0; j < corners_prior.size(); j++ )
	{
		cv::circle( print_prior, corners_prior[j], 5,  cv::Scalar(150), 2, 8, 0 );
	}
  
	//Prints
	cv::imshow("corners", print);
	cv::imshow("corners prior", print_prior);

	cv::waitKey(0);
	
	
	//******************** Fixing some points
	
	std::vector<cv::Point2f> fixed_points;
	
	cv::Point2f p1_slam;
	p1_slam.x = 294;
	p1_slam.y = 15;
	cv::Point2f p2_slam;
	p2_slam.x = 291;
	p2_slam.y = 79;
	
	cv::Point2f p1_prior;
	p1_prior.x = 288;
	p1_prior.y = 38;
	cv::Point2f p2_prior;
	p2_prior.x = 286;
	p2_prior.y = 102;
	
	fixed_points.push_back(p1_prior);
	fixed_points.push_back(p1_slam);
	fixed_points.push_back(p2_prior);
	fixed_points.push_back(p2_slam);
	
// 	auto it = ret_opencv.begin();
// 	for(it; it != ret_opencv.end() ; ++it){
// 		
// 	}
	
	//******************** Now associated the points
	
	std::cout << "TYPES : " << AASS::das::type2str(src_gray.type()) << " " << AASS::das::type2str(ndt_cv.type()) << std::endl;
	
// 	exit(0);
	
	cv::Mat homography;
		
	AASS::das::RANSACAssociation rcpd;
	
	rcpd.match(corners_prior, ret_opencv, homography, fixed_points, src_gray, ndt_cv);
	
	std::vector<cv::Point2f> corners_model = rcpd.getKeypointModel();
	std::vector<cv::Point2f> corners_moved = rcpd.getKeypointMoved();
	auto association = rcpd.getAssociations();
	
	cv::Mat draw;
	rcpd.draw(src_gray, ndt_cv, draw);
	
	cv::imshow("resultat RASNAC CPD:)", draw);
	cv::waitKey(0);
	
	
	//************************* BUILDING THE GRAPH********************//
	
	
// 	ndt_feature::G2OGraphMarker g2o_graph(g2o::SE2(0.2, 0.1, -0.1), 
// 						Eigen::Vector2d(0.05, 0.01), 
// 						DEG2RAD(2.),
// 						Eigen::Vector2d(0.05, 0.05),
// 						Eigen::Vector2d(0.05, 0.01),
// 						Eigen::Vector2d(0.2, 0.2));
// 	
// 	const ndt_feature::NDTFeatureGraphCorner ndt_graph;
// 	
// 	//First add ROBOT POSES and the landmarks :
// 	ndt_feature::ndtGraphToG2O(ndt_graph, g2o_graph);
// 	
// 	
// 	
// 	//Then add the priors landmarks and edges
// 	g2o_graph.addAllPriors(cornerDetect.getGraph());
// 	
// 	//Then add the links between the two
// 	g2o_graph.makeGraph();
// 	g2o_graph.save("prior_graph.g2o");
	
	
}