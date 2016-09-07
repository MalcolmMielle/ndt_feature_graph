#ifndef NDTFEATURE_PRIORAUTOCOMPLETE_05092016
#define NDTFEATURE_PRIORAUTOCOMPLETE_05092016

#include "das/RANSACAndCPD.hpp"
#include "das/RANSAC.hpp"
#include "das/priors/BasementPriorLine.hpp"
#include "das/NDTCorner.hpp"
#include "G2OGraphMaker.hpp"
#include "ndt_map/ndt_map.h"
#include "G2OGraphMaker.hpp"
#include "ndt_feature/ndt_feature_graph.h"


namespace ndt_feature{
	/**
	* @brief class where the whole autocomplete of the slam is produced
	* 
	*/
	class PriorAutoComplete{
		
	protected:
		lslgeneric::NDTMap _map;
		std::vector<cv::Point2f> _corner_prior;
		std::vector<cv::Point2f> _corner_prior_matched;
		std::vector<cv::Point2f> _ndt_corner_opencv;
		AASS::das::AssociationInterface _associations;
		cv::Mat _scale_transform;
		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge> _prior_graph;
		
	public:
		PriorAutoComplete(const std::string& file){
			
			//*****************************First load the map and calculate points
		// 	std::string file = "/home/malcolm/Documents/mapping0.jff";
		// 	map.loadFromJFF(file.c_str());
			
			
	// 		double resolution = 0.2;
	// 		auto mapGrid = new lslgeneric::LazyGrid(resolution);
	// 		lslgeneric::NDTMap map(mapGrid);
	// 		if(map.loadFromJFF(file.c_str()) < 0)
	// 			std::cout << "File didn't load" << std::endl;
	// 		std::cout << "File loaded" << std::endl;
		
			
		};
		
		
		/**
		* @brief Extracting the corner from the prior
		*/
		void extractCornerPrior(const std::string& file){
	// 		AASS::das::BasementPriorLine basement;
			cv::Mat src = cv::imread( file, CV_LOAD_IMAGE_COLOR ), src_gray;
			cv::cvtColor(src, src_gray, CV_RGB2GRAY );
			AASS::das::CornerDetector cornerDetect;
			cornerDetect.getFeaturesGraph(file);
			cornerDetect.removeClosePoints(20);
			_corner_prior = cornerDetect.getGraphPoint();
			_prior_graph = cornerDetect.getGraph();
			
		}
		
		/**
		* @brief Extracting the corner from the ndt_map
		*/
		void extractCornerNDT(const lslgeneric::NDTMap& ndt_map){
			AASS::das::NDTCorner corners;
			std::cout << "Searching for corners" << std::endl;
			_ndt_corner_opencv = corners.getAccurateCvCorners();
		}
		
		
		/**
		* @brief find homography between the images and transform the prior onto the slam
		*/
		void findScaleAndTransform(const std::vector< cv::Point2f >& src_base, const std::vector< cv::Point2f >& dst_base){
			_scale_transform = cv::findHomography(src_base, dst_base, CV_RANSAC, 3, cv::noArray());
			cv::perspectiveTransform( _corner_prior, _corner_prior_matched, _scale_transform);
		}
		
		/**
		* @brief Transform on corner map onto the other
		*/ 
		void transformOntoPriorMap(){
			//Scale transform here:		
			//Then create association
			std::vector<std::pair < cv::Point2f , cv::Point2f > > asso = _associations.associatedKeypoint2Closest(_corner_prior, _corner_prior_matched, _ndt_corner_opencv);
		}
		
		/**
		* @brief Create the optimization graph
		*/
		void createGraph(ndt_feature::NDTFeatureGraph& ndt_graph, G2OGraphMarker& g2o_graph){
			//Add NDTGraph robot poses and odometry
			//TODO: this function :(
			g2o_graph.addRobotPoseAndOdometry(ndt_graph);
			
			//Add landmark and observations
			//TODO: this function :(
			g2o_graph.addLandmarkAndObservation(ndt_graph);
			
			//Add prior graph
			g2o_graph.addAllPriors(_prior_graph);
			
			//Add links
			g2o_graph.addLinkBetweenMaps(_associations, -10);
		}
		
		//Massive TODO 
		
		void Optimize(){}
		
		void visualization(){}
		
	};
}

#endif