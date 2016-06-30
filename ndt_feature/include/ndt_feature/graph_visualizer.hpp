#ifndef NDTFEATURE_GRAPHVISUALIZER_30062016
#define NDTFEATURE_GRAPHVISUALIZER_30062016


#include "ndt_feature/ndt_feature_graph.h"
#include <visualization_msgs/Marker.h>

namespace ndt_feature {

	//TODO : Add the graph as an attributue later
	class GraphVisualizer{
		
	protected :
		
		
	public:
		GraphVisualizer(){
			
		};
		
		void rvizPrint(const NDTFeatureGraph& graph, visualization_msgs::Marker& origins){
			visualization_msgs::Marker destinations;
			origins.header.frame_id = destinations.header.frame_id = "/world";
			origins.header.stamp = destinations.header.stamp = ros::Time::now();
			origins.ns = destinations.ns = "graph_markers";
			origins.id = 0;
			destinations.id = 1;
			origins.type = destinations.type = visualization_msgs::Marker::POINTS;
			origins.scale.x = destinations.scale.x = 1;
			origins.scale.y = destinations.scale.y = 1;
			origins.color.g = destinations.color.r = 1.0f;
			origins.color.a = destinations.color.a = 1.0;
			
			std::cout << "Getting incre" << std::endl;
			std::vector<NDTFeatureLink> links = graph.getIncrementalLinks();
			std::cout << "Got incre" << std::endl;
			//Apply transformations.
			//Prints
			geometry_msgs::Point p;
			//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
			p.x = 0;
			p.y = 0;
			p.z = 0;
			origins.points.push_back(p);
			
			geometry_msgs::Point cumulated_translation = p;
			for(size_t i = 0; i < links.size() ; ++i){
				std::cout << "Making poitns " << links.size() << std::endl;
				//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
				Eigen::Affine3d T = links[i].T;
				cumulated_translation.x = cumulated_translation.x + T(0, 3);
				cumulated_translation.y = cumulated_translation.y + T(1, 3);
				cumulated_translation.z = 0;
				origins.points.push_back(cumulated_translation);
			}
			std::cout << "Point made.............." << std::endl;
// 			marker_pub.publish(points);
			
		}
		
		
	};
	
}

#endif