#include "das/RANSACAndCPD.hpp"
#include "das/RANSAC.hpp"
#include "das/priors/BasementPriorLine.hpp"
#include "G2OGraphMaker.hpp"
#include "vodigrex/utils/Utils.hpp"

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
	
	ndt_feature::G2OGraphMarker g2o_graph(g2o::SE2(0.2, 0.1, -0.1), 
						Eigen::Vector2d(0.05, 0.01), 
						DEG2RAD(2.),
						Eigen::Vector2d(0.05, 0.05),
						Eigen::Vector2d(0.05, 0.01),
						Eigen::Vector2d(0.2, 0.2));
	
	g2o_graph.addAllPriors(cornerDetect.getGraph());
	g2o_graph.makeGraph();
	g2o_graph.save("prior_graph.g2o");
	
	cv::Mat moved = cv::Mat::zeros(src.size(), src.type());
	
	auto graphh = cornerDetect.getGraph();
	AASS::vodigrex::draw(graphh, moved);
	
	cv::imshow("GRAPH", moved);
	cv::waitKey(0);
	
// 	 cv::Mat print;
// 	src_gray.copyTo(print);
// 	
// 	for( int j = 0; j < corners_prior.size(); j++ )
// 	{
// 		cv::circle( print, corners_prior[j], 5,  cv::Scalar(150), 2, 8, 0 );
// 	}
// 	
//   
//   //Prints
//   cv::imshow("corners", print);
//   cv::waitKey(0);
// 	
// // 	std::cout << " argv[1] " << argv[1] << std::endl;
//   /// Load source image and convert it to gray
// //   cv::Mat src = cv::imread( argv[1], 1 ), src_gray;
// //   cv::cvtColor( src, src_gray, CV_BGR2GRAY );
//   
// 	cv::Mat src_gray_rotated = rotate(src_gray, 180);
//   
// //   cv::Mat src_rot = cv::imread( argv[2], 1 ), src_gray_rotated;
// //   cv::cvtColor( src_rot, src_gray_rotated, CV_BGR2GRAY );
// 	
// 
//  
//   
//   std::vector<cv::Point2f> fixed_points;
//   
// // cv::Point2f p1;
// // p1.x = 177;
// // p1.y = 15;
// // cv::Point2f p1_model;
// // p1_model.x = 177;
// // p1_model.y = 15;
// // 
// // cv::Point2f p2;
// // p2.x = 260;
// // p2.y = 15;
// // cv::Point2f p2_model;
// // p2_model.x = 260;
// // p2_model.y = 15;
//   
//   
// //   Rotate!!!!
// //   cv::Point2f p1;
// //   p1.x = 326;
// //   p1.y = 479;
// //   cv::Point2f p1_model;
// //   p1_model.x = 177;
// //   p1_model.y = 15;
// //   
// //   cv::Point2f p2;
// //   p2.x = 243;
// //   p2.y = 475;
// //   cv::Point2f p2_model;
// //   p2_model.x = 260;
// //   p2_model.y = 15;
// //   
// 
//   
//   
//   
//   
//   //Fixing the border :
//   
// 	cv::Point2f p1;
// 	p1.x = 0;
// 	p1.y = 0;
// 	cv::Point2f p2;
// 	p2.x = src_gray_rotated.rows;
// 	p2.y = src_gray_rotated.cols;
// 	cv::Point2f p1_model;
// 	p1_model.x = 0;
// 	p1_model.y = 0;
// 	cv::Point2f p2_model;
// 	p2_model.x = src_gray.rows;
// 	p2_model.y = src_gray.cols;
// 	
// 	fixed_points.push_back(p1);
// 	fixed_points.push_back(p1_model);
// 
// 	fixed_points.push_back(p2);
// 	fixed_points.push_back(p2_model);
// 	
// 	cv::Mat homography;
// 		
// 	AASS::das::RANSACAndCPD rcpd;
// 	
// // 	const std::vector<cv::Point2f>& data, const std::vector<cv::Point2f>& model, cv::Mat& homography, const std::vector<cv::Point2f>& fixed_points = std::vector<cv::Point2f>(), const cv::Mat& data_gray = cv::Mat(), const cv::Mat model_gray = cv::Mat()
// 	
// 	rcpd.match(corners_prior, corners_prior, homography, fixed_points, src_gray, src_gray);
// // 	rcpd.match(src_gray_rotated, src_gray, homography, fixed_points);
// 	
// 	std::vector<cv::Point2f> corners = rcpd.getKeypointModel();
// 	std::vector<cv::Point2f> corners_rotated = rcpd.getKeypointMoved();
// 
//   
//   std::cout << "CPD DONE" << std::endl;
//   std::cout << "Homography : " << std::endl << homography << std::endl;
//   
// // //   std::vector<cv::Point2f> scene_corners(corners_rotated.size());
// //   cv::perspectiveTransform( corners_rotated, scene_corners, homography);
//   
// 	cv::Mat moved = cv::Mat::zeros(src.size(), src.type());
// 
// 	for(size_t i = 0 ; i < corners_rotated.size() ; ++i){
// 	// 								std::cout << "DRAWING KEYPOINT" <<data_base[i] << std::endl;
// 		cv::circle(moved, corners_rotated[i], 5, cv::Scalar(255));
// 	}
// 	cv::Mat still = cv::Mat::zeros(src.size(), src.type());
// 	for(size_t i = 0 ; i < corners.size() ; ++i){
// // 								std::cout << "DRAWING KEYPOINT" <<data_base[i] << std::endl;
// 		cv::circle(still, corners[i], 5, cv::Scalar(255));
// 	}
// //   
// 
// 	std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;
// 	std::vector< cv::DMatch > good_matches;
// 	cv::Mat moooo;
// 	cv::drawMatches( still, keypoints_object, moved, keypoints_scene,
// 		good_matches, moooo, cv::Scalar::all(-1), cv::Scalar::all(-1),
// 		std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
// 	
// 	cv::imshow("RES FINAL", moooo);
// 	
// 
// 	cv::Mat links;
// 	rcpd.draw(src_gray, src_gray_rotated, links);
// // 	auto associations = rcpd.getAssociations();
// // 	
// 
// // 	cv::drawMatches( src_gray, keypoints_object, src_gray_rotated, keypoints_scene,
// // 				good_matches, links, cv::Scalar::all(-1), cv::Scalar::all(-1),
// // 				std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
// // 	
// // 	for(size_t i = 0 ; i < associations.size() ; ++i){
// // 		cv::Point2f p2_model = associations[i].second;
// // 		cv::Point2f p1 = associations[i].first;
// // 		p1.x = p1.x + src_gray.cols;
// // 		cv::line(links, p2_model, p1, cv::Scalar(255));
// // 	}
// 	
// 
// 	cv::imshow("Lnks", links);  
//   
// 	cv::waitKey(0);
// 	
// 	std::cout << "Byebye " << std::endl;
  
}