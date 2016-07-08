#ifndef NDTFEATURE_GRAPHG2O3D_08072016
#define NDTFEATURE_GRAPHG2O3D_08072016

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"
// #include "types_tutorial_slam2d.h"


#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "ndt_feature/ndt_feature_graph.h"
#include "ndt_feature/utils.h"

namespace ndt_feature {
	
	class G2OGraphOptimization3d {
		
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
		G2OGraphOptimization3d() : cleanFmt(4, 0, ", ", "\n", "[", "]"){
			
			_linearSolver = new SlamLinearSolver();
			_linearSolver->setBlockOrdering(false);
			_blockSolver = new SlamBlockSolver(_linearSolver);
			_solver = new g2o::OptimizationAlgorithmGaussNewton(_blockSolver);
			_linearSolver->setBlockOrdering(false);
			_optimizer.setAlgorithm(_solver);

			// add the parameter representing the sensor offset
			g2o::ParameterSE3Offset* sensorOffset = new g2o::ParameterSE3Offset;
			//TODO Add an isometry for sensor offset
// 			g2o::SE3 sensorOffsetTransf(0.2, 0.1, -0.1);
			sensorOffset->setOffset(/*sensorOffsetTransf*/);
			sensorOffset->setId(0);
			_optimizer.addParameter(sensorOffset);
		};
		
		
		~G2OGraphOptimization3d(){
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
			g2o::VertexSE3* firstRobotPose = dynamic_cast<g2o::VertexSE3*>(_optimizer.vertex(0));
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
				auto links_odom = graph.getOdometryLinks();
				
				for(auto tmp = 0 ; tmp < links_odom.size() ; ++tmp){
				Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
					std::cout <<"Estimate before anything " << links_odom[tmp].getRelCov().inverse().format(cleanFmt) << std::endl;
					std::cout << "Adding Edge" << std::endl;	
				}
				
// 				graph.appendLinks(links_odom);
				std::cout << graph.getNbNodes() - 1 << " ? ==" <<  links_odom.size() << std::endl;
				assert(graph.getNbNodes() - 1 == links_odom.size() );
				addRobotPoses(graph);
				addOdometry(links_odom);
				std::cout << "SAVE" << std::endl;
				_optimizer.save("tutorial_real_final.g2o");
				std::cout << "Done saving " << std::endl;
			}

			
		}
		
		/**
		 * @brief Add edges between Robot poses
		 */
		void addOdometry(const std::vector<NDTFeatureLink>& links){
			
			for (size_t i = 0 ; i <links.size() ; ++i) {
				Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
				std::cout <<"Estimate before anything " << links[i].getRelCov().inverse().format(cleanFmt) << std::endl;
				std::cout << "Adding Edge" << std::endl;
// 				NDTFeatureLink link = NDTFeatureLink((const NDTFeatureLink&) graph.getLinkInterface(i));
				g2o::EdgeSE3* odometry = new g2o::EdgeSE3;
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
				g2o::VertexSE3* robot =  new g2o::VertexSE3;
				robot->setId(i);
				NDTFeatureNode2VertexSE2(*feature, *robot);
				std::cout << "Robot " << robot->estimate().matrix() << std::endl;
				_optimizer.addVertex(robot);
				auto Vet_test = (g2o::VertexSE3*) _optimizer.vertex(i);
				std::cout << "SE2 " << Vet_test->estimate().matrix() << std::endl;
				
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
		
		void NDTFeatureLink2EdgeSE2(const NDTFeatureLink& link, g2o::EdgeSE3& edge);
		void NDTFeatureNode2VertexSE2(const ndt_feature::NDTFeatureNode& feature, g2o::VertexSE3& vertex_se3);
		
		Eigen::Isometry3d Affine3d2Isometry3d(const Eigen::Affine3d& affine);
		
		
	};

	
	
	inline void G2OGraphOptimization3d::NDTFeatureLink2EdgeSE2(const NDTFeatureLink& link, g2o::EdgeSE3& edge)
	{
		size_t from = link.getRefIdx() ;
		size_t toward = link.getMovIdx() ;
		
		std::cout << "from " << from << " toward " << toward << std::endl;
		
		edge.vertices()[0] = _optimizer.vertex(from);
		edge.vertices()[1] = _optimizer.vertex(toward);
		
		Eigen::Affine3d affine = link.getRelPose();	
		
		
		Eigen::Isometry3d isometry3d = Affine3d2Isometry3d(affine);
// 		double x = cumulated_translation(0, 3);
// 		double y = cumulated_translation(1, 3);
// 		g2o::SE3 se3(isometry3d);
		
		edge.setMeasurement(isometry3d);
		
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
		
		
		//HACK ? : HOW IT'S DONE IN G2o. Also maybe look here : https://robotics.stackexchange.com/questions/7960/how-to-generate-edges-for-pose-graph-slam-from-known-positions-and-loops
// 		Eigen::Vector2d transNoise(0.05, 0.01);
// 		//Deg to radian
// 		double rotNoise = 2. * 0.01745329251994329575;
// 
// 		Eigen::Matrix3d covariance;
// 		covariance.fill(0.);
// 		
// 		std::cout <<"Estimate of zero " << covariance.format(cleanFmt) << std::endl;
// 		covariance(0, 0) = transNoise[0]*transNoise[0];
// 		covariance(1, 1) = transNoise[1]*transNoise[1];
// 		covariance(2, 2) = rotNoise*rotNoise;
// 		
// 		std::cout <<"Estimate of covariance " << covariance.format(cleanFmt) <<  std::endl << " With rot " << rotNoise*rotNoise << std::endl;
// 		Eigen::Matrix3d information = covariance.inverse();
// 		
// // 		Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "\t", " ", "", "", "", "");
// 		
// 		std::cout <<"Estimate of information " << information.format(cleanFmt) << std::endl;
// // 		std::cout <<"Estimate " << link.getRelCov().inverse().format(cleanFmt) << std::endl;
// 
// 		edge.setInformation(information);

	}
	
	inline void G2OGraphOptimization3d::NDTFeatureNode2VertexSE2(const ndt_feature::NDTFeatureNode& feature, g2o::VertexSE3& vertex_se3)
	{
		Eigen::Affine3d affine = Eigen::Affine3d(feature.getPose());
// 		getRelPose();		
		Eigen::Isometry3d isometry3d = Affine3d2Isometry3d(affine);
		
		Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "\t", " ", "", "", "", "");
		std::cout <<"Estimate " << isometry3d.matrix().format(cleanFmt) << std::endl;
// 		double x = cumulated_translation(0, 3);
// 		double y = cumulated_translation(1, 3);
		
		std::cout << "SE2 " << isometry3d.matrix() << std::endl;
		vertex_se3.setEstimate(isometry3d);
		
		std::cout << "Vector in func " << vertex_se3.estimate().matrix() << std::endl;
	}

	
	inline Eigen::Isometry3d G2OGraphOptimization3d::Affine3d2Isometry3d(const Eigen::Affine3d& affine){
		
		Eigen::Isometry3d isometry3d;
		isometry3d.translation() = affine.translation();
		isometry3d.linear() = affine.rotation();		
		return isometry3d;
		
	}

		
		
}


#endif