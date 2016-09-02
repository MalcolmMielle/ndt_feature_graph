#ifndef NDTFEATURE_GRAPHVISUALIZER_30062016
#define NDTFEATURE_GRAPHVISUALIZER_30062016


#include "ndt_feature/ndt_feature_graph.h"
#include <visualization_msgs/Marker.h>
#include <ndt_map/NDTMapMsg.h>
#include <ndt_map/ndt_conversions.h>
#include <ndt_map/ndt_map.h>

namespace ndt_feature {

	//TODO : Add the graph as an attributue later
	class GraphVisualizer{
		
	protected :
		
		int _count;
		int _node;
		Eigen::Affine3d _start_pose;
		ndt_feature::NDTFeatureGraph* _graph;
		
	public:
		GraphVisualizer() : _count(0), _node(0){
			
		};
		
		void setGraph(ndt_feature::NDTFeatureGraph* graph){_graph = graph;};
		
		void loadGraph(const std::string &fileName, int nb_nodes = -1){
			if(nb_nodes < 0){
				throw std::runtime_error("Not engouh nodes");
			}
			_graph->load(fileName, nb_nodes);
		}
		
		void printLoadedGraph(){
			visualization_msgs::Marker origins, origins_odom;
			ndt_map::NDTMapMsg mapmsg;
			const std::string frame_name = "/graph_visualizer";
			
			//Print all node center
			rvizPrint(*_graph, origins, origins_odom, frame_name);
			
			//For all node print the cloud
			for(size_t i = 0; i < _graph->getNbNodes() ; ++i){
				lslgeneric::NDTMap* map = _graph->getMap();
				bool good = lslgeneric::toMessage(map, mapmsg, frame_name);
			}
			
			
			
		}
		
		void setStart(const Eigen::Affine3d& in){
			_start_pose = in;
		}
		
		void printAll(NDTFeatureGraph& graph, visualization_msgs::Marker& origins, visualization_msgs::Marker& origins_odom, ndt_map::NDTMapMsg& mapmsg, const std::string& frame_name){
			
// 			_count++;
			rvizPrint(graph, origins, origins_odom, frame_name);
			
// 			if(_count % 10 == 0){
// 				_node++;
// 			}
// 			if(_node==6) _node = 0;
// 			std::cout << " count and node " << _count << " " << _node << std::endl;
			
			//Get the last ndtMap element
			lslgeneric::NDTMap* map = graph.getMap(_node);
			
// 			lslgeneric::NDTMap* map = map->pseudoTransformNDTMap();
			bool good = lslgeneric::toMessage(map, mapmsg, frame_name);
			
// 			TODO Change it to the fuser frame ?
// 			printNDTMap(, mapmsg, frame_name);
			
		}
		
		/**
		 * @brief Only print red square at every node position
		 */
		void rvizPrint(const NDTFeatureGraph& graph, visualization_msgs::Marker& origins, visualization_msgs::Marker& origins_odom){
			rvizPrint(graph, origins, origins_odom, "/fuser");
		}
		
		void rvizPrint(const NDTFeatureGraph& graph, visualization_msgs::Marker& origins, visualization_msgs::Marker& origins_odom, const std::string& frame){
			origins.header.frame_id = frame;
			origins.header.stamp = ros::Time::now();
			origins.ns = "graph_markers";
			origins.id = 0;
			origins.type = visualization_msgs::Marker::POINTS;
			origins.scale.x = 0.2;
			origins.scale.y = 0.2;
			origins.color.r = 1.0f;
			origins.color.a = 1.0;
			
			origins_odom.header.frame_id = frame;
			origins_odom.header.stamp = ros::Time::now();
			origins_odom.ns = "graph_markers";
			origins_odom.id = 0;
			origins_odom.type = visualization_msgs::Marker::POINTS;
			origins_odom.scale.x = 0.2;
			origins_odom.scale.y = 0.2;
			origins_odom.color.b = 1.0f;
			origins_odom.color.a = 1.0;
			
// 			std::cout << "Getting incre :" << graph.getNbLinks() << std::endl;
			std::vector<NDTFeatureLink> links_odom = graph.getOdometryLinks();
			std::vector<NDTFeatureLink> links = graph.getIncrementalLinks();
// 			std::cout << "Got incre" << std::endl;
			//Apply transformations.
			//Prints
			
			if(graph.getNbNodes() == 1 && links_odom.size() > 0){
				std::cout << "WRONG LINKS" << std::endl;
				exit(0);
			}
			
			
			printLinks(links_odom, origins_odom);
			printLinks(links, origins);
			
		}
		
		void printLinks(const std::vector<NDTFeatureLink>& links, visualization_msgs::Marker& origins){
			std::cout << "Got incre" << std::endl;
			std::ofstream ofs;
			ofs.open("allpositions.txt", std::ofstream::out | std::ofstream::trunc);
			//Apply transformations.
			//Prints
			geometry_msgs::Point p;
			//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
			p.x = _start_pose(0, 3);
			p.y = _start_pose(1, 3);
// 			p.x = 0;
// 			p.y = 0;
			p.z = 0;
			origins.points.push_back(p);
			ofs << p.x << " " << p.y << std::endl;
			
// 			geometry_msgs::Point cumulated_translation = p;
			
			Eigen::Affine3d cumulated_translation = _start_pose;
			
			for(size_t i = 0; i < links.size() ; ++i){
// 				std::cout << "Making poitns " << links.size() << std::endl;
				//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
				Eigen::Affine3d T = links[i].T;
				
				cumulated_translation = cumulated_translation * T;
				
				
				p.x = cumulated_translation(0, 3);
				p.y = cumulated_translation(1, 3);
				p.z = 0;
				ofs << p.x << " " << p.y << std::endl;
				origins.points.push_back(p);
				
				std::cout << " Translation " << links[i].T.translation() << std::endl;
				std::cout << " Rotation " << links[i].T.rotation() << std::endl;
			}
			ofs.close();
			exit(0);
// 			std::cout << "Point made.............." << std::endl;
// 			marker_pub.publish(points);
		}
		
	
		void printNDTMap(const ndt_feature::NDTFeatureNode& node, ndt_map::NDTMapMsg& mapmsg, std::string frame_name){
			lslgeneric::NDTMap* map = node.map->map;
			bool good = lslgeneric::toMessage(map, mapmsg, frame_name);
		
		};
		
		
	};
	
}

#endif