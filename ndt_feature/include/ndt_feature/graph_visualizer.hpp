#ifndef NDTFEATURE_GRAPHVISUALIZER_30062016
#define NDTFEATURE_GRAPHVISUALIZER_30062016


#include "ndt_feature/ndt_feature_graph.h"
#include <visualization_msgs/Marker.h>

namespace ndt_feature {

	//TODO : Add the graph as an attributue later
	class GraphVisualizer{
		
	protected :
		
		Eigen::Affine3d _start_pose;
		
	public:
		GraphVisualizer(){
			
		};
		
		void setStart(const Eigen::Affine3d& in){
			_start_pose = in;
		}
		
		/**
		 * @brief Only print red square at every node position
		 */
		void rvizPrint(const NDTFeatureGraph& graph, visualization_msgs::Marker& origins){
			origins.header.frame_id = "/world";
			origins.header.stamp = ros::Time::now();
			origins.ns = "graph_markers";
			origins.id = 0;
			origins.type = visualization_msgs::Marker::POINTS;
			origins.scale.x = 0.2;
			origins.scale.y = 0.2;
			origins.color.r = 1.0f;
			origins.color.a = 1.0;
			
			std::cout << "Getting incre" << std::endl;
			std::vector<NDTFeatureLink> links = graph.getIncrementalLinks();
			std::cout << "Got incre" << std::endl;
			//Apply transformations.
			//Prints
			geometry_msgs::Point p;
			//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
			p.x = _start_pose(0, 3);
			p.y = _start_pose(1, 3);
			p.z = 0;
			origins.points.push_back(p);
			
// 			geometry_msgs::Point cumulated_translation = p;
			
			Eigen::Affine3d cumulated_translation = _start_pose;
			for(size_t i = 0; i < links.size() ; ++i){
				std::cout << "Making poitns " << links.size() << std::endl;
				//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
				Eigen::Affine3d T = links[i].T;
				
				cumulated_translation = cumulated_translation * T;
				
				
				p.x = cumulated_translation(0, 3);
				p.y = cumulated_translation(1, 3);
				p.z = 0;
				origins.points.push_back(p);
			}
			std::cout << "Point made.............." << std::endl;
// 			marker_pub.publish(points);
			
		}
		
		
	};
	
}

#endif