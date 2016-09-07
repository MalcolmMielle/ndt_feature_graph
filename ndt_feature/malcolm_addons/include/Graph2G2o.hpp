#ifndef NDTFEATURE_GRAPHG2O_01072016
#define NDTFEATURE_GRAPHG2O_01072016

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"
#include "g2o/types/slam2d/edge_se2_prior.h"
#include "g2o/types/slam2d/edge_se2_link.h"
// #include "types_tutorial_slam2d.h"


#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "ndt_feature/ndt_feature_graph.h"
#include "ndt_feature/utils.h"

#include "bettergraph/PseudoGraph.hpp"
#include "vodigrex/linefollower/SimpleNode.hpp"

#include "das/AssociationInterface.hpp"
#include "covariance.hpp"

namespace ndt_feature {
	
	class G2OGraphOptimization {
		
	public :
		typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
		typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
	protected:
		g2o::SparseOptimizer _optimizer;
		SlamLinearSolver* _linearSolver;
		SlamBlockSolver* _blockSolver;
		g2o::OptimizationAlgorithmGaussNewton* _solver;
		Eigen::IOFormat cleanFmt;
		
		
	public:
		// :( I hate pointers
		G2OGraphOptimization() : cleanFmt(4, 0, ", ", "\n", "[", "]"){
			
			_linearSolver = new SlamLinearSolver();
			_linearSolver->setBlockOrdering(false);
			_blockSolver = new SlamBlockSolver(_linearSolver);
			_solver = new g2o::OptimizationAlgorithmGaussNewton(_blockSolver);
			_linearSolver->setBlockOrdering(false);
			_optimizer.setAlgorithm(_solver);

			// add the parameter representing the sensor offset
			g2o::ParameterSE2Offset* sensorOffset = new g2o::ParameterSE2Offset;
			g2o::SE2 sensorOffsetTransf(0.2, 0.1, -0.1);
			sensorOffset->setOffset(sensorOffsetTransf);
			sensorOffset->setId(0);
			_optimizer.addParameter(sensorOffset);
		};
		
		
		~G2OGraphOptimization(){
			// freeing the graph memory -> Need real new pointer because of this
			_optimizer.clear();
			// destroy all the singletons
			g2o::Factory::destroy();
			g2o::OptimizationAlgorithmFactory::destroy();
			g2o::HyperGraphActionLibrary::destroy();
		}
		
		void optimize(){
			// dump initial state to the disk
			_optimizer.save("tutorial_before_optimize_real.g2o");

			// prepare and run the optimization
			// fix the first robot pose to account for gauge freedom
			g2o::VertexSE2* firstRobotPose = dynamic_cast<g2o::VertexSE2*>(_optimizer.vertex(0));
			firstRobotPose->setFixed(true);
			_optimizer.setVerbose(true);
			_optimizer.save("ndt_graph.g2o");
			
			std::cerr << "Optimizing" << std::endl;
			_optimizer.initializeOptimization();
			_optimizer.optimize(10);
			std::cerr << "done." << std::endl;
		
			_optimizer.save("tutorial_after_optimize_real.g2o");
		
			// freeing the graph memory
			_optimizer.clear();
			
			
		}
		
		void updateGraph(NDTFeatureGraph& graph){
			std::cout << "Updating the graph : " << graph.getNbNodes() << " " << graph.getNbLinks() << std::endl;
			_optimizer.clear();
			if(graph.getNbNodes() > 0){
				//FIrst update Links
				
				addRobotPoses(graph);
// 				addOdometry(graph);
				addLinkRegistration(graph);
				std::cout << "SAVE" << std::endl;
				_optimizer.save("tutorial_real_final.g2o");
				std::cout << "Done saving " << std::endl;
			}

			
		}
		
		/**
		 * @brief Add edges between Robot poses
		 */
		void addOdometry(NDTFeatureGraph& graph){
			
			auto links = graph.getOdometryLinks();
				
			for(auto tmp = 0 ; tmp < links.size() ; ++tmp){
				Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
				std::cout <<"Estimate before anything " << links[tmp].getRelCov().inverse().format(cleanFmt) << std::endl;
				std::cout << "Adding Edge" << std::endl;	
			}
			
// 				graph.appendLinks(links_odom);
			std::cout << graph.getNbNodes() - 1 << " ? ==" <<  links.size() << std::endl;
			assert(graph.getNbNodes() - 1 == links.size() );
			
			for (size_t i = 0 ; i <links.size() ; ++i) {
				Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
				std::cout <<"Estimate before anything " << links[i].getRelCov().inverse().format(cleanFmt) << std::endl;
				std::cout << "Adding Edge" << std::endl;
// 				NDTFeatureLink link = NDTFeatureLink((const NDTFeatureLink&) graph.getLinkInterface(i));
				g2o::EdgeSE2* odometry = new g2o::EdgeSE2;
				NDTFeatureLink2EdgeSE2(links[i], *odometry);
				_optimizer.addEdge(odometry);
			} 
			std::cout << "SAVE now" << std::endl;
			_optimizer.save("tutorial_links.g2o");
			
		}
		
		void addLinkRegistration(NDTFeatureGraph& graph){
			
			//What number ot use for the neighbor
			auto links = graph.getOdometryLinks();
			graph.updateLinksUsingNDTRegistration(links, 10, true);
			
			for (size_t i = 0 ; i <links.size() ; ++i) {
				Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
				std::cout <<"Estimate before anything " << links[i].getRelCov().inverse().format(cleanFmt) << std::endl;
				std::cout << "Adding Edge" << std::endl;
// 				NDTFeatureLink link = NDTFeatureLink((const NDTFeatureLink&) graph.getLinkInterface(i));
				g2o::EdgeSE2* odometry = new g2o::EdgeSE2;
				NDTFeatureLink2EdgeSE2(links[i], *odometry);
				_optimizer.addEdge(odometry);
			} 
			std::cout << "SAVE now" << std::endl;
			_optimizer.save("tutorial_links.g2o");
			
		}
		
		/**
		 * @brief Add all the robot poses as vertices
		 */
		void addRobotPoses(const NDTFeatureGraph& graph){
			
			int size_before = _optimizer.vertices().size();
			 
			for (size_t i = 0; i < graph.getNbNodes(); ++i) {
				std::cout << "Adding node " << i << std::endl;
				NDTFeatureNode* feature = new NDTFeatureNode();
				feature->copyNDTFeatureNode( (const NDTFeatureNode&)graph.getNodeInterface(i) );
				g2o::VertexSE2* robot =  new g2o::VertexSE2;
				robot->setId(i);
				NDTFeatureNode2VertexSE2(*feature, *robot);
				std::cout << "Robot " << robot->estimate().toVector() << std::endl;
				_optimizer.addVertex(robot);
				auto Vet_test = (g2o::VertexSE2*) _optimizer.vertex(i);
				std::cout << "SE2 " << Vet_test->estimate().toVector() << std::endl;
				
			}
			
			//Make sure the convertion worked
			if(graph.getNbNodes() != _optimizer.vertices().size() - size_before ){
				std::cout << "Merde" << std::endl;
			}
			assert(graph.getNbNodes() == ( _optimizer.vertices().size() - size_before ) );
			std::cout << " DONE WITH VERT " << _optimizer.vertices().size() << std::endl;
			std::cout << "SAVE now" << std::endl;
			_optimizer.save("tutorial_vertex.g2o");
		}
		
		/**
		 * @brief Add all landmark detected as vertices
		 */
		void addLandmark(){}
		
		/**
		 * @brief Add all observations of landmarks
		 */
		void addObservation(){}
		
	private:
		
		void NDTFeatureLink2EdgeSE2(const NDTFeatureLink& link, g2o::EdgeSE2& edge, bool useCov = false);
		void NDTFeatureNode2VertexSE2(const ndt_feature::NDTFeatureNode& feature, g2o::VertexSE2& vertex_se2);
		
		Eigen::Isometry2d Affine3d2Isometry2d(const Eigen::Affine3d& affine);
		
		
	};

	
	
	inline void G2OGraphOptimization::NDTFeatureLink2EdgeSE2(const NDTFeatureLink& link, g2o::EdgeSE2& edge, bool useCov)
	{
		size_t from = link.getRefIdx() ;
		size_t toward = link.getMovIdx() ;
		
		std::cout << "from " << from << " toward " << toward << std::endl;
		
		edge.vertices()[0] = _optimizer.vertex(from);
		edge.vertices()[1] = _optimizer.vertex(toward);
		
		Eigen::Affine3d affine = link.getRelPose();		
		Eigen::Isometry2d isometry2d = Affine3d2Isometry2d(affine);
// 		double x = cumulated_translation(0, 3);
// 		double y = cumulated_translation(1, 3);
		g2o::SE2 se2(isometry2d);
		
		edge.setMeasurement(se2);
		
		//HOW IT SHOUDL BE DONE :
// 		//Information is covariance
// 		Eigen::Matrix3d cov = link.getRelCov().inverse();
// 		
// 		Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "\t", " ", "", "", "", "");
// 		Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
// 		std::cout <<"Estimate " << cov.format(cleanFmt) << std::endl;
// 		std::cout <<"Estimate " << link.getRelCov().inverse().format(cleanFmt) << std::endl;
		
		
		//BUG : Covariance of each node : USE SCAN MATCHING
// 		auto cov_fo_node = node.map.getCov();
		
		Eigen::Matrix3d covariance;
		if(useCov == false){
			//HACK ? : HOW IT'S DONE IN G2o. Also maybe look here : https://robotics.stackexchange.com/questions/7960/how-to-generate-edges-for-pose-graph-slam-from-known-positions-and-loops
			Eigen::Vector2d transNoise(0.05, 0.01);
			//Deg to radian
			double rotNoise = 2. * 0.01745329251994329575;

			
			covariance.fill(0.);
			
			std::cout <<"Estimate of zero " << covariance.format(cleanFmt) << std::endl;
			covariance(0, 0) = transNoise[0]*transNoise[0];
			covariance(1, 1) = transNoise[1]*transNoise[1];
			covariance(2, 2) = rotNoise*rotNoise;
			
			std::cout <<"Estimate of covariance " << covariance.format(cleanFmt) <<  std::endl << " With rot " << rotNoise*rotNoise << std::endl;
		}
		else{
			covariance = link.getRelCov();
		}
		Eigen::Matrix3d information = covariance.inverse();
		
// 		Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "\t", " ", "", "", "", "");
		
		std::cout <<"Estimate of information " << information.format(cleanFmt) << std::endl;
// 		std::cout <<"Estimate " << link.getRelCov().inverse().format(cleanFmt) << std::endl;

		edge.setInformation(information);

	}
	
	inline void G2OGraphOptimization::NDTFeatureNode2VertexSE2(const NDTFeatureNode& feature, g2o::VertexSE2& vertex_se2)
	{
		Eigen::Affine3d affine = Eigen::Affine3d(feature.getPose());
// 		getRelPose();		
		Eigen::Isometry2d isometry2d = Affine3d2Isometry2d(affine);
		
		Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "\t", " ", "", "", "", "");
		std::cout <<"Estimate " << isometry2d.matrix().format(cleanFmt) << std::endl;
// 		double x = cumulated_translation(0, 3);
// 		double y = cumulated_translation(1, 3);
		g2o::SE2 se2(isometry2d);
		
		std::cout << "SE2 " << se2.toVector() << std::endl;
		vertex_se2.setEstimate(se2);
		
		std::cout << "Vector in func " << vertex_se2.estimate().toVector() << std::endl;
	}

	
	inline Eigen::Isometry2d G2OGraphOptimization::Affine3d2Isometry2d(const Eigen::Affine3d& affine){
		
		Eigen::Affine2d affine2d = lslgeneric::eigenAffine3dTo2d(affine);
		Eigen::Isometry2d isometry2d;
		isometry2d.translation() = affine2d.translation();
		isometry2d.linear() = affine2d.rotation();
		return isometry2d;
		
	}

		
		
}


#endif