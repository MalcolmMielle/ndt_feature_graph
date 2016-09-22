#ifndef NDTFEATURE_ACG_15102016
#define NDTFEATURE_ACG_15102016

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"
#include "g2o/types/slam2d/edge_se2_prior.h"
#include "g2o/types/slam2d/edge_se2_link.h"
#include "g2o/types/slam2d/edge_landmark_se2.h"
#include "g2o/types/slam2d/edge_link_xy.h"
// #include "types_tutorial_slam2d.h"


#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "ndt_feature/ndt_feature_graph.h"
#include "ndt_feature/utils.h"

#include "Eigen/Core"

#include "bettergraph/PseudoGraph.hpp"
#include "vodigrex/linefollower/SimpleNode.hpp"

#include "das/AssociationInterface.hpp"
#include "das/NDTCorner.hpp"
#include "covariance.hpp"

namespace ndt_feature {
	
	
	class VertexPrior : public g2o::VertexSE2{
		
	};

	
	
	class AutoCompleteGraph{
	protected:
		
		///@brief vector storing all node from the prior 
		std::vector<ndt_feature::VertexPrior*> _nodes_prior;
		///@brief vector storing all node from the prior 
		std::vector<g2o::VertexPointXY*> _nodes_landmark;
		///@brief vector storing all node from the ndt ndt_feature_graph
		std::vector<g2o::VertexSE2*> _nodes_ndt;
		///@brief vector storing all linking edges
		std::vector<g2o::EdgeLinkXY_malcolm*> _edge_link;
		///@brief vector storing all edges between a landmark and the robot
		std::vector<g2o::EdgeSE2Landmark_malcolm*> _edge_landmark;
		///@brief vector storing all edge between the prior nodes
		std::vector<g2o::EdgeSE2Prior_malcolm*> _edge_prior;
		///@brief vector storing the odometry
		std::vector<g2o::EdgeSE2*> _edge_odometry;
		
		///@brief the main dish : the graph
		g2o::OptimizableGraph _optimizable_graph;
		
		
	public:
		
		AutoCompleteGraph(){}
		
		
		/***FUNCTIONS TO ADD THE NODES***/
		g2o::VertexSE2* addRobotPose(const g2o::SE2& se2){
			
			g2o::VertexSE2* robot =  new g2o::VertexSE2;
			robot->setEstimate(se2);
			robot->setId(_optimizable_graph.vertices().size());
			_optimizable_graph.addVertex(robot);
			_nodes_ndt.push_back(robot);
			return robot;
		}
		g2o::VertexSE2* addRobotPose(const Eigen::Vector3d& rob){
			g2o::SE2 se2(rob(0), rob(1), rob(2));
			return addRobotPose(se2);
		}
		g2o::VertexSE2* addRobotPose(double x, double y, double theta){
			Eigen::Vector3d robot1;
			robot1 << x, y, theta;
			return addRobotPose(robot1);
		}
		
		g2o::VertexPointXY* addLandmarkPose(const g2o::Vector2D& pos, int strength = 1){
			g2o::VertexPointXY* landmark = new g2o::VertexPointXY;
			landmark->setId(_optimizable_graph.vertices().size());
			landmark->setEstimate(pos);
			_optimizable_graph.addVertex(landmark);
			_nodes_landmark.push_back(landmark);
			return landmark;
		}
		g2o::VertexPointXY* addLandmarkPose(double x, double y, int strength = 1){
			g2o::Vector2D lan;
			lan << x, y;
			return addLandmarkPose(lan, strength);
		}
		
		ndt_feature::VertexPrior* addPriorLandmarkPose(const g2o::SE2& se2){
			ndt_feature::VertexPrior* priorlandmark = new ndt_feature::VertexPrior;
			priorlandmark->setId(_optimizable_graph.vertices().size());
			priorlandmark->setEstimate(se2);
			_optimizable_graph.addVertex(priorlandmark);
			_nodes_prior.push_back(priorlandmark);
			return priorlandmark;
		}
		ndt_feature::VertexPrior* addPriorLandmarkPose(const Eigen::Vector3d& lan){
			g2o::SE2 se2(lan(0), lan(1), lan(2));
			return addPriorLandmarkPose(se2);
		}
		ndt_feature::VertexPrior* addPriorLandmarkPose(double x, double y, double theta){
			Eigen::Vector3d lan;
			lan << x, y, theta;
			return addPriorLandmarkPose(lan);
		}
		
		
		/** FUNCTION TO ADD THE EGDES **/
		
		void addOdometry(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2){
			g2o::EdgeSE2* odometry = new g2o::EdgeSE2;
			odometry->vertices()[0] = v1 ;
			odometry->vertices()[1] = v2 ;
			odometry->setMeasurement(se2);
			odometry->setInformation(information);
			_optimizable_graph.addEdge(odometry);
			_edge_odometry.push_back(odometry);
		}
		void addOdometry(const g2o::SE2& observ, int from, int toward){
			g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from);
			g2o::HyperGraph::Vertex* toward_ptr = _optimizable_graph.vertex(toward);
			addOdometry(observ, from_ptr, toward_ptr);
		}
		void addOdometry(double x, double y, double theta, int from, int toward){
			g2o::SE2 se2(x, y, theta);
			addOdometry(se2, from, toward);
		}
		
		void addLandmarkObservation(const g2o::Vector2D& pos, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2){
			g2o::EdgeSE2PointXY* landmarkObservation =  new g2o::EdgeSE2PointXY;
			landmarkObservation->vertices()[0] = v1;
			landmarkObservation->vertices()[1] = v2;
			landmarkObservation->setMeasurement(pos);
			landmarkObservation->setInformation(information_landmark);
			landmarkObservation->setParameterId(0, _sensorOffset->id());
			_optimizable_graph.addEdge(landmarkObservation);
			_edge_landmark.push_back(landmarkObservation);
		}
		void addLandmarkObservation(const g2o::Vector2D& pos, int from, int toward){
			g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from);
			g2o::HyperGraph::Vertex* toward_ptr = _optimizable_graph.vertex(toward);
			addLandmarkObservation(pos, from_ptr, toward_ptr);
		}
		
		void addEdgePrior(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2){
			g2o::EdgeSE2Prior_malcolm* priorObservation =  new g2o::EdgeSE2Prior_malcolm;
			priorObservation->vertices()[0] = v1;
			priorObservation->vertices()[1] = v2;
			priorObservation->setMeasurement(se2);
			priorObservation->setInformation(information_prior);
			priorObservation->setParameterId(0, _sensorOffset->id());
			_optimizable_graph.addEdge(priorObservation);
			_edge_prior.push_back(priorObservation);
		}
		void addEdgePrior(g2o::SE2 observ, int from, int toward){
			std::tuple<g2o::SE2, int, int> obs1(observ, from, toward);
			addEdgePrior(obs1);
		}
		void addEdgePrior(double x, double y, double theta, int from, int toward){
			g2o::SE2 se2(x, y, theta);
			addEdgePrior(se2, from, toward);
			
		}
		
		void addLinkBetweenMaps(const g2o::Vector2D& pos, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2){
			g2o::EdgeLinkXY_malcolm* linkObservation =  new g2o::EdgeLinkXY_malcolm;
			linkObservation->vertices()[0] = v1;
			linkObservation->vertices()[1] = v2;
			linkObservation->setMeasurement(pos);
			linkObservation->setInformation(information_link);
			linkObservation->setParameterId(0, _sensorOffset->id());
			_optimizable_graph.addEdge(linkObservation);
			_edge_link.push_back(linkObservation);
		}
		void addLinkBetweenMaps(const g2o::Vector2D& pos, int from, int toward){
			g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from);
			g2o::HyperGraph::Vertex* toward_ptr = _optimizable_graph.vertex(toward);
			addLinkBetweenMaps(pos, from_ptr, toward_ptr);
		}
		
		
		//FUNCTION TO REMOVE A VERTEX
		void removeVertex(g2o::HyperGraph::Vertex* v1){
			//Prior
			if( typeid(v1) == typeid(ndt_feature::VertexPrior*)){
				int index = findPriorNode(v1);
				assert(index != -1);
				std::vector<ndt_feature::VertexPrior*>::iterator which = _nodes_prior.begin() + index;
				_nodes_prior.erase(which);
			}
			//Robot node
			if( typeid(v1) == typeid(g2o::VertexSE2*)){
				int index = findRobotNode(v1);
				assert(index != -1);
				std::vector<ndt_feature::VertexPrior*>::iterator which = _nodes_ndt.begin() + index;
				_nodes_ndt.erase(which);
				
			}
			//Landmark Node
			if( typeid(v1) == typeid(g2o::VertexPointXY*)){
				int index = findLandmarkNode(v1);
				assert(index != -1);
				std::vector<ndt_feature::VertexPrior*>::iterator which = _nodes_landmark.begin() + index;
				_nodes_landmark.erase(which);
			}
			
			_optimizable_graph.removeVertex(v1, true);
		}
		
		int findRobotNode(g2o::HyperGraph::Vertex* v){
			int pos = 0;
			auto it = _nodes_ndt.begin();
			for(it ; it != _nodes_ndt.end() ; ++it){
				if(*it == v){
					return pos;
				}
				++pos;
			}
			return -1;
		}
		int findLandmarkNode(g2o::HyperGraph::Vertex* v){
			int pos = 0;
			auto it = _nodes_landmark.begin();
			for(it ; it != _nodes_landmark.end() ; ++it){
				if(*it == v){
					return pos;
				}
				++pos;
			}
			return -1;
			
		}
		int findPriorNode(g2o::HyperGraph::Vertex* v){
			int pos = 0;
			auto it = _nodes_prior.begin();
			for(it ; it != _nodes_prior.end() ; ++it){
				if(*it == v){
					return pos;
				}
				++pos;
			}
			return -1;
			
		}
		
		
		//FUNTION TO ADD A PRIOR GRAPH INTO THE GRAPH
		void addPriorGraph(const bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>& graph){
			
			std::pair< bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::VertexIterator, bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::VertexIterator > vp;
			std::deque<bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex> vec_deque;
			std::vector<ndt_feature::VertexPrior*> out_prior;
			
			for (vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
				bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex v = *vp.first;
				//ATTENTION Magic number
				ndt_feature::VertexPrior* res = addPriorLandmarkPose(graph[v].getX(), graph[v].getY(), 0);
				vec_deque.push_back(v);
				out_prior.push_back(res);
			}
			
			int count = 0;
			for (vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
				bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex v = *vp.first;
				bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::EdgeIterator out_i, out_end;
				bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Edge e;
				
				for (boost::tie(out_i, out_end) = boost::out_edges(v, (graph)); 
					out_i != out_end; ++out_i) {
					e = *out_i;
					bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex targ = boost::target(e, (graph));
				
					int idx = -1;
					for(size_t ii = count +1 ; ii < vec_deque.size() ; ++ii){
						if(targ == vec_deque[ii]){
							idx = ii;
						}
					}
					if(idx == -1){
						//SKIP
					}
					else{
						ndt_feature::VertexPrior* from = out_prior[count];
						ndt_feature::VertexPrior* toward = out_prior[idx];
						int x_diff = graph[targ].getX() - graph[v].getX();
						int y_diff = graph[targ].getY() - graph[v].getY();
						g2o::SE2 se2(x_diff, y_diff, 0);
						addEdgePrior(se2, from, toward);
					}
				
				}
				++count;
			}
			
			std::cout << _edge_prior.size() << " == " << graph.getNumEdges() << std::endl;
			assert( _nodes_landmark.size() == graph.getNumVertices() );
			assert( _edge_prior.size() == graph.getNumEdges());
			
		}
		
	
	};
}

#endif