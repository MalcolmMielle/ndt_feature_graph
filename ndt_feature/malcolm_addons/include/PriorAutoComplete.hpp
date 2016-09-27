#ifndef NDTFEATURE_PRIORAUTOCOMPLETE_05092016
#define NDTFEATURE_PRIORAUTOCOMPLETE_05092016

#include "das/RANSACAndCPD.hpp"
#include "das/RANSAC.hpp"
#include "das/priors/BasementPriorLine.hpp"
#include "das/NDTCorner.hpp"
#include "das/RANSACAssociation.hpp"
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
		
		//TODO : should be one element
		std::vector<cv::Point2f> _ndt_corner_opencv;
		std::vector<NDTCornerGraphElement> _ndt_corner_opencv_and_nodes;
		
		
		AASS::das::AssociationInterface _associations;
		//HACK : hardcoded
		std::vector<cv::Point2f> _same_point_prior;
		std::vector<cv::Point2f> _same_point_slam;
		
		/** Scale from slam to prior**/
		cv::Mat _scale_transform;
		cv::Mat _scale_transform_prior2ndt;
		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge> _prior_graph;
		
	public:
		PriorAutoComplete(){
			
			_same_point_prior.push_back(cv::Point2f(287,99));
			_same_point_slam.push_back(cv::Point2f(19.36, 6.25));
			
			_same_point_prior.push_back(cv::Point2f(287,42));                   
			_same_point_slam.push_back(cv::Point2f(19.14,2.25));
			
// 			_scale_transform_prior2ndt = (cv::Mat_<double>(3,3) << -0.2786094788480143, 0.3498731413928963, -15.10862915834429, -0.2991881900376893, -0.1765065659037397, 83.83182860356848, -0.02088327087513238, 0.01890914775038407, 1);
			
			//ATTENTION : next line or for used the points only
			
			_same_point_prior.push_back(cv::Point2f(117,37));
			_same_point_slam.push_back(cv::Point2f(7.64,3.63));
			
			
			
			_same_point_prior.push_back(cv::Point2f(138, 260));              
			_same_point_slam.push_back(cv::Point2f(9.5, 16));
			
			//bad point :(
// 			_same_point_prior.push_back(cv::Point2f(287,70));              
// 			_same_point_slam.push_back(cv::Point2f(19.25,4.25));  
			
			_scale_transform = cv::findHomography(_same_point_slam, _same_point_prior, CV_RANSAC, 3, cv::noArray());
			_scale_transform_prior2ndt = cv::findHomography(_same_point_prior, _same_point_slam, CV_RANSAC, 3, cv::noArray());
		
		};
		
		
		/**
		* @brief Extracting the corner from the prior
		*/
		void extractCornerPrior(const std::string& file){
	// 		AASS::das::BasementPriorLine basement;
			cv::Mat src = cv::imread( file, CV_LOAD_IMAGE_COLOR ), src_gray;
			cv::cvtColor(src, src_gray, CV_RGB2GRAY );
			
			
			cv::threshold(src_gray, src_gray, 100, 255, src_gray.type());
			
// 			cv::imshow("src", src_gray);
// 			cv::waitKey(0);
			AASS::das::CornerDetector cornerDetect;
			cornerDetect.getFeaturesGraph(file);
			cornerDetect.removeClosePoints(20);
			_corner_prior = cornerDetect.getGraphPoint();
			_prior_graph = cornerDetect.getGraph(); 
			
			
			
			
		}
		
		/**
		* @brief Extracting the corner from the ndt_map
		*/
		void extractCornerNDT(ndt_feature::NDTFeatureGraph& ndt_graph){
			
			_ndt_corner_opencv.clear();
			_ndt_corner_opencv_and_nodes.clear();
			
			//For every node in the graph : extract NDT map
			//ADDING ROBOT POSES
			std::cout << "/******************************** LANDMARKS " << std::endl << std::endl;
			
			double cell_size = 0;
			
			for (size_t inode_number = 0; inode_number < ndt_graph.getNbNodes(); ++inode_number) {
				std::cout << "Adding landmark for node " << inode_number << std::endl;
// 				NDTFeatureNode* feature = new NDTFeatureNode();
// 				feature->copyNDTFeatureNode( (const NDTFeatureNode&)ndt_graph.getNodeInterface(i) );
// // 				g2o::SE2 robot_se2 = NDTFeatureNode2VertexSE2(*feature);
				
				lslgeneric::NDTMap* map = ndt_graph.getMap(inode_number);
				
				//HACK For now : we translate the Corner extracted and not the ndt-maps
				auto cells = map->getAllCells();
				double x2, y2, z2;
				map->getCellSizeInMeters(x2, y2, z2);
				cell_size = x2;
				
				AASS::das::NDTCorner cornersExtractor;
				std::cout << "Searching for corners in map with " << cells.size() << " initialized cells, and celle size is " << x2 << " " << y2 << " " << z2 << std::endl;
				auto ret_export = cornersExtractor.getAllCorners(*map);
				auto ret_opencv_point_corner = cornersExtractor.getAccurateCvCorners();			
				std::cout << "Corner extracted. Nb of them " << ret_opencv_point_corner.size() << std::endl;
				
				//HACK: translate the corners now :
				auto it = ret_opencv_point_corner.begin();
				
				std::vector<NDTCornerGraphElement> _final_corners;
				
				for(it ; it != ret_opencv_point_corner.end() ; ++it){
					std::cout << "MOVE : "<< it -> x << " " << it-> y << std::endl;
					Eigen::Vector3d vec;
					vec << it->x, it->y, 0;
					Eigen::Vector3d vec_out = ndt_graph.getNode(inode_number).T * vec;
					NDTCornerGraphElement p_out(vec_out(0), vec_out(1));
					//Push the number of node
					p_out.push_back(inode_number);
					
					std::cout << "NEW POINT : "<< p_out.point << std::endl;
					_final_corners.push_back(p_out);
				}
				
				//Copy them back to opencv
				
				//Extract the corners
				auto it_cor = _final_corners.begin();
				int count = 0 ;
				
				std::cout << " ********************************************* NDT CORNERS : "<< std::endl;
				
				for(it_cor ; it_cor != _final_corners.end() ; ++it_cor){
					
					_ndt_corner_opencv_and_nodes.push_back(*it_cor);
					
					std::cout << cv::Point2f(it_cor->point.x, it_cor->point.y) << std::endl;
					
// 					addLandmarkPose(it->x, it->y, 0);
// 					Eigen::Vector2d real_obs ;
// 					real_obs << it->x, it->y;
// 					Eigen::Vector2d observation;
// 					//Projecting real_obs into robot coordinate frame
// 					Eigen::Vector2d trueObservation = _robot_positions[i].inverse() * real_obs;
// 					observation = trueObservation;
// 					std::cout << "G2O OBSERVATION " << observation << std::endl;
// 					//HACK magic number
// 					g2o::SE2 se2(observation(0), observation(1), 0);
// 					std::cout << "G2O OBSERVATION " << se2.toVector() << std::endl;
// 					std::tuple<g2o::SE2, int, int> tup(se2, i, _robot_positions.size() + nb_of_landmark + count);
// 					_observation_real_landmarks.push_back(tup);
// 					count++;
					
				}
				
				std::cout << std::endl << " //////////------> "<<count << " corners here " << std::endl;
			}
			
			
			
			//Clear the corner. Indeed they have been cleared in the node but not in between the getNbNodes
			std::vector < NDTCornerGraphElement > tmp;
			for(size_t i = 0 ; i < _ndt_corner_opencv_and_nodes.size() ; ++i){
				std::cout << "data : " ;  _ndt_corner_opencv_and_nodes[i].print() ; std::cout << std::endl;	
				bool seen = false;
				int ind = -1;
				for(size_t j = 0 ; j < tmp.size() ; ++j){
		// 			if(tmp[j] == _corners_position[i]){
					double res = cv::norm(tmp[j].point - _ndt_corner_opencv_and_nodes[i].point);
					
					std::cout << "res : " << res << " points "  << tmp[j].point << " " << _ndt_corner_opencv_and_nodes[i].point << "  cell size " << cell_size << std::endl;
					
					if( res < cell_size){
						seen = true;
						ind = j;
					}
				}
				if(seen == false){	
					std::cout << "New point" << std::endl;
					tmp.push_back(_ndt_corner_opencv_and_nodes[i]);
				}
				else{
					std::cout << "Point seen " << std::endl;
					tmp[ind].addNodes(_ndt_corner_opencv_and_nodes[i]);
				}
			}
	
			_ndt_corner_opencv_and_nodes.clear();
			_ndt_corner_opencv_and_nodes = tmp;
			
			
			
			std::cout << "********************** NDT CORNER OF TRIMMING" << std::endl;
			for(size_t i = 0; i < _ndt_corner_opencv_and_nodes.size() ; ++i){
				std::cout << "data : " ;  _ndt_corner_opencv_and_nodes[i].print() ; std::cout << std::endl;	
			}
			
			std::cout << std::endl << std::endl;
			
// 			exit(0);
			
			//HACK copy
			for(size_t i = 0; i < _ndt_corner_opencv_and_nodes.size() ; ++i){
				_ndt_corner_opencv.push_back(cv::Point2f(_ndt_corner_opencv_and_nodes[i].point.x, _ndt_corner_opencv_and_nodes[i].point.y));
			}
			
			
			
				//Add the landmarks
		}
		
		
		void printAsso(){
			std::cout << "Print asso" << std::endl;
			
			for(size_t i = 0; i < _associations.getAssociations().size() ; ++i){
				std::cout << "data : " <<  _associations.getAssociations()[i].first << " model " << _associations.getAssociations()[i].second << std::endl;
				
			}
		}
		
		/**
		* @brief find homography between the images and transform the prior onto the slam
		*/
		void findScale(){
// 			_scale_transform = cv::findHomography(src_base, dst_base, CV_RANSAC, 3, cv::noArray());
// 			cv::perspectiveTransform( _corner_prior, _corner_prior_matched, _scale_transform);
			
			std::cout << "Searching for the scale " << std::endl;
			cv::Mat homography;
			AASS::das::RANSAC rcpd;
			
			assert(_same_point_prior.size () == _same_point_slam.size());
			std::vector<cv::Point2f> fixed_point;
			for(size_t i = 0 ; i < _same_point_prior.size() ; ++i){
				fixed_point.push_back(_same_point_prior[i]);
				fixed_point.push_back(_same_point_slam[i]);
			}
			
			std::cout << "**********************CORNER PRIOR BEFORE RANSAC" << std::endl;
			for(size_t i = 0; i < _corner_prior.size() ; ++i){
				std::cout << "data : " <<  _corner_prior[i] << std::endl;
				
			}
			
			std::cout << "**********************CORNER NDT BEFORE RANSAC" << std::endl;
			for(size_t i = 0; i < _ndt_corner_opencv.size() ; ++i){
				std::cout << "data : " <<  _ndt_corner_opencv[i] << std::endl;
				
			}
			
			std::cout << "Let's gop ransac " << std::endl;
			rcpd.ransac(_corner_prior, _ndt_corner_opencv, homography, fixed_point);
			std::cout << "Ransac DONE homography type : " << homography << std::endl;
			_scale_transform_prior2ndt = homography;
			
			exit(0);
		}
		
		/**
		* @brief Transform on corner map onto the other
		*/ 
		void transformOntoSLAM(){
			
			std::cout << "Transform onto slam from scale : " << std::endl << _scale_transform_prior2ndt << std::endl;
			//Scale transform here:		
			//Then create association
			cv::perspectiveTransform( _corner_prior, _corner_prior_matched, _scale_transform_prior2ndt);
			
			//ATTENTION here we change the data for the moved for clarity in g2o_viewer. that's why we need to resize the graph 11 lines away
			std::vector<std::pair < cv::Point2f , cv::Point2f > > asso = _associations.associatedKeypoint2Closest(_corner_prior_matched, _corner_prior_matched, _ndt_corner_opencv);
			_associations.setAssociations(asso);
			_associations.setKeypointData(_corner_prior);
			_associations.setKeypointModel(_ndt_corner_opencv);
			_associations.setKeypointMoved(_corner_prior_matched);
			std::cout << "Association number " << _associations.getAssociations().size() << std::endl;
			std::cout << "Association number " << asso.size() << std::endl;
			std::cout << "Corner number " << _corner_prior.size() << std::endl;
			std::cout << "NDT number " << _ndt_corner_opencv.size() << std::endl;
			std::cout << "Prior number " << _corner_prior_matched.size() << std::endl;
			
			std::cout << "Printing the asociations" << std::endl;
			printAsso();
			
			
			//Scale graph
			std::pair< AASS::das::CornerDetector::CornerVertexIterator, AASS::das::CornerDetector::CornerVertexIterator > vp;
			//vertices access all the vertix
			//Classify them in order
// 				std::cout << "Gph size : " << _graph.getNumVertices() << std::endl;
			int i = 0 ;
			for (vp = boost::vertices(_prior_graph); vp.first != vp.second; ++vp.first) {
// 					std::cout << "going throught grph " << i << std::endl; ++i;
				AASS::das::CornerDetector::CornerVertex v = *vp.first;
				cv::Point2f p;
				std::cout << "OLD value " << _prior_graph[v].getX() << " " << _prior_graph[v].getY() << std::endl;
				std::cout << "New value " << _corner_prior_matched[i].x << " " << _corner_prior_matched[i].y << std::endl;
				_prior_graph[v].setX(_corner_prior_matched[i].x);
				_prior_graph[v].setY(_corner_prior_matched[i].y);
				std::cout << "New value " << _prior_graph[v].getX() << " " << _prior_graph[v].getY() << std::endl;
				++i;
			}
			
// 			exit(0);
// 			
		}
		
		/**
		* @brief Create the optimization graph
		*/
		void createGraphLinked(ndt_feature::NDTFeatureGraph& ndt_graph, G2OGraphMarker& g2o_graph){
			
			std::cout << std::endl << "CREATE GRAPH" << std::endl << std::endl;
			//Add NDTGraph robot poses and odometry
			g2o_graph.addRobotPoseAndOdometry(ndt_graph);
			
			//Add landmark and observations
// 			g2o_graph.addLandmarkAndObservation(ndt_graph);
			g2o_graph.addLandmarkAndObservation(_ndt_corner_opencv_and_nodes);
			
			//Add prior graph
			g2o_graph.addAllPriors(_prior_graph);
			
			//Add links
			std::cout << "Printing the asociations" << std::endl;
			printAsso();
			g2o_graph.addLinkBetweenMaps(_associations, -10);
			
			g2o_graph.makeGraph();
		}
		
		/**
		* @brief Create the optimization graph
		*/
		void createGraphLinkedOriented(ndt_feature::NDTFeatureGraph& ndt_graph, G2OGraphMarker& g2o_graph){
			
			std::cout << std::endl << "CREATE GRAPH" << std::endl << std::endl;
			//Add NDTGraph robot poses and odometry
			g2o_graph.addRobotPoseAndOdometry(ndt_graph);
			
			//Add landmark and observations
// 			g2o_graph.addLandmarkAndObservation(ndt_graph);
			g2o_graph.addLandmarkAndObservation(_ndt_corner_opencv_and_nodes);
			
			//Add prior graph
			g2o_graph.addAllPriors(_prior_graph);
			
			//Add links
			std::cout << "Printing the asociations" << std::endl;
			printAsso();
			g2o_graph.addLinkBetweenMaps(_associations, -10);
			
			g2o_graph.makeGraphAllOriented();
		}
		
		void createGraphLinkedOrientedNoPrior(ndt_feature::NDTFeatureGraph& ndt_graph, G2OGraphMarker& g2o_graph){
			
			std::cout << std::endl << "CREATE GRAPH" << std::endl << std::endl;
			//Add NDTGraph robot poses and odometry
			g2o_graph.addRobotPoseAndOdometry(ndt_graph);
			
			//Add landmark and observations
// 			g2o_graph.addLandmarkAndObservation(ndt_graph);
			g2o_graph.addLandmarkAndObservation(_ndt_corner_opencv_and_nodes);
			
			//Add prior graph
// 			g2o_graph.addAllPriors(_prior_graph);
			
			//Add links
			std::cout << "Printing the asociations" << std::endl;
// 			printAsso();
// 			g2o_graph.addLinkBetweenMaps(_associations, -10);
			
			g2o_graph.makeGraphAllOriented();
		}
		
		
		/**
		* @brief Create the optimization graph
		*/
		void createGraph(ndt_feature::NDTFeatureGraph& ndt_graph, G2OGraphMarker& g2o_graph){
			
			std::cout << std::endl << "CREATE GRAPH" << std::endl << std::endl;
			//Add NDTGraph robot poses and odometry
// 			g2o_graph.addRobotPoseAndOdometry(ndt_graph);
			
			//Add landmark and observations
// 			g2o_graph.addLandmarkAndObservation(ndt_graph);
// 			g2o_graph.addLandmarkAndObservation(_ndt_corner_opencv_and_nodes);
			
			//Add prior graph
			g2o_graph.addAllPriors(_prior_graph);
			
			//Add links
			std::cout << "Printing the asociations" << std::endl;
			printAsso();
// 			g2o_graph.addLinkBetweenMaps(_associations, -10);
			
			g2o_graph.makeGraph();
		}
		//Massive TODO 
		
// 		void Optimize(){}
		
// 		void visualization(){}
		
	};
}

#endif