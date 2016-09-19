#ifndef NDTFEATURE_ACG_15102016
#define NDTFEATURE_ACG_15102016

#include <ctime>

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
	
	class NDTCornerGraphElement{
	public:
		cv::Point2f point;
	protected:
		//TODO : change it to a set
		std::vector<int> nodes_linked;
		std::vector<g2o::VertexSE2*> nodes_linked_ptr;
		std::vector<g2o::Vector2D> observations;
		
	public:
		NDTCornerGraphElement(float x, float y) : point(x, y){};
		NDTCornerGraphElement(const cv::Point2f& p) : point(p){};
		
		void addAllObserv(int i, g2o::VertexSE2* ptr, g2o::Vector2D obs){
			nodes_linked.push_back(i);
			observations.push_back(obs);
			nodes_linked_ptr.push_back(ptr);
		}
		
		size_t size(){
			assert(nodes_linked.size() == nodes_linked_ptr.size());
			assert(nodes_linked.size() == observations.size());
			return nodes_linked.size();
		}
		
// 		std::vector<int>& getNodeLinked(){return nodes_linked;}
		const std::vector<int>& getNodeLinked() const {return nodes_linked;}
		const std::vector<g2o::VertexSE2*>& getNodeLinkedPtr() const {return nodes_linked_ptr;}
		const std::vector<g2o::Vector2D>& getObservations() const {return observations;}
		
// 		void push_back(int i){nodes_linked.push_back(i);}
		
// 		void addNode(int i){nodes_linked.push_back(i);}
// 		void addObservation(const g2o::Vector2D& obs){ observations.push_back(obs);}
		
		void fuse(const NDTCornerGraphElement& cor){
			for(size_t i = 0 ; i < cor.getNodeLinked().size() ; ++i){
				bool seen = false;
				for(size_t j = 0 ; j < nodes_linked.size() ; ++j){
					if(cor.getNodeLinked()[i] == nodes_linked[j]){
						seen = true;
					}
				}
				if(seen == false){
					nodes_linked.push_back(cor.getNodeLinked()[i]);
					observations.push_back(cor.getObservations()[i]);
					nodes_linked_ptr.push_back(cor.getNodeLinkedPtr()[i]);
				}
			}
		}
		void print() const {std::cout << point << " nodes : ";
			
			for(size_t i = 0 ; i < nodes_linked.size()  ; ++i){
				std::cout << nodes_linked[i] << " " ;
			}
			
		}
		
	};

	
	
	class AutoCompleteGraph{
	protected:
		
		//Needed system values
		Eigen::Vector2d _transNoise;
		double _rotNoise;
		Eigen::Vector2d _landmarkNoise;
		Eigen::Vector2d _priorNoise;
		Eigen::Vector2d _linkNoise;
		g2o::ParameterSE2Offset* _sensorOffset;
		g2o::SE2 _sensorOffsetTransf;
		
		///@brief vector storing all node from the prior 
		std::vector<ndt_feature::VertexPrior*> _nodes_prior;
		///@brief vector storing all node from the prior 
		std::vector<g2o::VertexPointXY*> _nodes_landmark;
		///@brief vector storing all node from the ndt ndt_feature_graph
		std::vector<g2o::VertexSE2*> _nodes_ndt;
		///@brief vector storing all linking edges
		std::vector<g2o::EdgeLinkXY_malcolm*> _edge_link;
		///@brief vector storing all edges between a landmark and the robot
		std::vector<g2o::EdgeSE2PointXY*> _edge_landmark;
		///@brief vector storing all edge between the prior nodes
		std::vector<g2o::EdgeSE2Prior_malcolm*> _edge_prior;
		///@brief vector storing the odometry
		std::vector<g2o::EdgeSE2*> _edge_odometry;
		
		///@brief the main dish : the graph
		g2o::OptimizableGraph _optimizable_graph;
		
		//ATTENTION : I should avoid that if I want to run both thread at the same time since no copy is made. I should instead copy it
		ndt_feature::NDTFeatureGraph* _ndt_graph;
		
// 		std::vector < NDTCornerGraphElement > _ndt_corners;
		
		
	private:
		int _previous_number_of_node_in_ndtgraph;
		
		
	public:
		
		AutoCompleteGraph(const g2o::SE2& sensoffset, 
						const Eigen::Vector2d& tn, 
						double rn,
						const Eigen::Vector2d& ln,
						const Eigen::Vector2d& pn,
						const Eigen::Vector2d& linkn,
						ndt_feature::NDTFeatureGraph* ndt_graph
  					) : _sensorOffsetTransf(sensoffset), _transNoise(tn), _rotNoise(rn), _landmarkNoise(ln), _priorNoise(pn), _linkNoise(linkn), _previous_number_of_node_in_ndtgraph(0), _ndt_graph(ndt_graph){
						// add the parameter representing the sensor offset ATTENTION was ist das ?
						_sensorOffset = new g2o::ParameterSE2Offset;
						_sensorOffset->setOffset(_sensorOffsetTransf);
						_sensorOffset->setId(0);
					}
		
		
		/** Accessor**/
		std::vector<ndt_feature::VertexPrior*>& getPriorNodes(){return _nodes_prior;}
		const std::vector<ndt_feature::VertexPrior*>& getPriorNodes() const {return _nodes_prior;}
		///@brief vector storing all node from the prior 
		std::vector<g2o::VertexPointXY*>& getLandmarkNodes(){return _nodes_landmark;}
		const std::vector<g2o::VertexPointXY*>& getLandmarkNodes() const {return _nodes_landmark;}
		///@brief vector storing all node from the ndt ndt_feature_graph
		std::vector<g2o::VertexSE2*>& getRobotNodes(){return _nodes_ndt;}
		const std::vector<g2o::VertexSE2*>& getRobotNodes() const {return _nodes_ndt;}
		///@brief vector storing all linking edges
		std::vector<g2o::EdgeLinkXY_malcolm*>& getLinkEdges(){return _edge_link;}
		const std::vector<g2o::EdgeLinkXY_malcolm*>& getLinkEdges() const {return _edge_link;}
		///@brief vector storing all edges between a landmark and the robot
		std::vector<g2o::EdgeSE2PointXY*>& getLandmarkEdges(){return _edge_landmark;}
		const std::vector<g2o::EdgeSE2PointXY*>& getLandmarkEdges() const {return _edge_landmark;}
		///@brief vector storing all edge between the prior nodes
		std::vector<g2o::EdgeSE2Prior_malcolm*>& getPriorEdges(){ return _edge_prior;}
		const std::vector<g2o::EdgeSE2Prior_malcolm*>& getPriorEdges() const { return _edge_prior;}
		///@brief vector storing the odometry
		std::vector<g2o::EdgeSE2*>& getOdometryEdges(){return _edge_odometry;}
		const std::vector<g2o::EdgeSE2*>& getOdometryEdges() const {return _edge_odometry;}
		
		///@brief the main dish : the graph
		g2o::OptimizableGraph& getGraph(){return _optimizable_graph;}
		const g2o::OptimizableGraph& getGraph() const {return _optimizable_graph;}
		
		/***FUNCTIONS TO ADD THE NODES***/
		g2o::VertexSE2* addRobotPose(const g2o::SE2& se2);
		g2o::VertexSE2* addRobotPose(const Eigen::Vector3d& rob);
		g2o::VertexSE2* addRobotPose(double x, double y, double theta);
		
		g2o::VertexPointXY* addLandmarkPose(const g2o::Vector2D& pos, int strength = 1);
		g2o::VertexPointXY* addLandmarkPose(double x, double y, int strength = 1);
		
		ndt_feature::VertexPrior* addPriorLandmarkPose(const g2o::SE2& se2);
		ndt_feature::VertexPrior* addPriorLandmarkPose(const Eigen::Vector3d& lan);
		ndt_feature::VertexPrior* addPriorLandmarkPose(double x, double y, double theta);
		
		
		/** FUNCTION TO ADD THE EGDES **/
		
		void addOdometry(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2);
		void addOdometry(const g2o::SE2& observ, int from, int toward);
		void addOdometry(double x, double y, double theta, int from, int toward);
		
		void addLandmarkObservation(const g2o::Vector2D& pos, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2);
		void addLandmarkObservation(const g2o::Vector2D& pos, int from, int toward);
		
		void addEdgePrior(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2);
// 		void addEdgePrior(g2o::SE2 observ, int from, int toward);
// 		void addEdgePrior(double x, double y, double theta, int from, int toward);
		
		void addLinkBetweenMaps(const g2o::Vector2D& pos, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2);
		void addLinkBetweenMaps(const g2o::Vector2D& pos, int from, int toward);
		
		
		//FUNCTION TO REMOVE A VERTEX
		void removeVertex(g2o::HyperGraph::Vertex* v1);
		
		int findRobotNode(g2o::HyperGraph::Vertex* v);
		int findLandmarkNode(g2o::HyperGraph::Vertex* v);
		int findPriorNode(g2o::HyperGraph::Vertex* v);
		
		
		//FUNTION TO ADD A PRIOR GRAPH INTO THE GRAPH
		
		/**
		 * @brief Directly use the prior graph to init the prior part of the ACG
		 * 
		 */
		void addPriorGraph(const bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>& graph);
		
		
		
		void copyNDTGraph(ndt_feature::NDTFeatureGraph& ndt_graph){
			
			//ATTENTION : Might crash
			if(_ndt_graph->wasInit()){
				delete _ndt_graph;
			}
			_ndt_graph = new ndt_feature::NDTFeatureGraph(ndt_graph);
			
		}
		
		
		
		void updateNDTGraph(){
			updateNDTGraph(*_ndt_graph);
		}
		
		/**
		 * @brief : take the NDT graph and update the NDT corners by adding every new node since last time and all new observations.
		 */
		void updateNDTGraph(ndt_feature::NDTFeatureGraph& ndt_graph);
		
	private:
		
		//TODO
		void updateLinksAfterNDTGraph(const std::vector<g2o::VertexPointXY*>& new_landmarks); 
		
		
	
	};
}

#endif