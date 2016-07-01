#ifndef NDTFEATURE_GRAPHG2O_01072016
#define NDTFEATURE_GRAPHG2O_01072016

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"
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
	
	class G2OGraphOptimization {
		
	public :
		typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
		typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
	protected:
		g2o::SparseOptimizer _optimizer;
		SlamLinearSolver* _linearSolver;
		SlamBlockSolver* _blockSolver;
		g2o::OptimizationAlgorithmGaussNewton* _solver;
		
		
	public:
		// :( I hate pointers
		G2OGraphOptimization(){
			
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
			// freeing the graph memory
			_optimizer.clear();
			// destroy all the singletons
			g2o::Factory::destroy();
			g2o::OptimizationAlgorithmFactory::destroy();
			g2o::HyperGraphActionLibrary::destroy();
		}
		
		void optimize(){
			// dump initial state to the disk
			_optimizer.save("tutorial_before_real.g2o");

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
		
			_optimizer.save("tutorial_after_real.g2o");
		
			// freeing the graph memory
			_optimizer.clear();
			
			
		}
		
		void updateGraph(const NDTFeatureGraph& graph){
			addRobotPoses(graph);
			addOdometry(graph);
		}
		
		/**
		 * @brief Add edges between Robot poses
		 */
		void addOdometry(const NDTFeatureGraph& graph){
			
			for (size_t i = 0; i < graph.getNbLinks(); ++i) {
				
				NDTFeatureLink link = NDTFeatureLink((const NDTFeatureLink&) graph.getLinkInterface(i));
				g2o::EdgeSE2* odometry = new g2o::EdgeSE2;
				NDTFeatureLink2EdgeSE2(link, *odometry);
				_optimizer.addEdge(odometry);
			}
			
		}
		
		/**
		 * @brief Add all the robot poses as vertices
		 */
		void addRobotPoses(const NDTFeatureGraph& graph){
			
			int size_before = _optimizer.vertices().size();
			
			for (size_t i = 0; i < graph.getNbNodes(); ++i) {
				
				NDTFeatureNode feature;
				feature.copyNDTFeatureNode( (const NDTFeatureNode&)graph.getNodeInterface(i) );
				g2o::VertexSE2* robot =  new g2o::VertexSE2;
				robot->setId(i);
				NDTFeatureNode2VertexSE2(feature, *robot);
				_optimizer.addVertex(robot);				
			}
			
			//Make sure the convertion worked
			assert(graph.getNbNodes() == ( _optimizer.vertices().size() - size_before ) );
			
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
		
		void NDTFeatureLink2EdgeSE2(const NDTFeatureLink& link, g2o::EdgeSE2& edge);
		void NDTFeatureNode2VertexSE2(const ndt_feature::NDTFeatureNode& feature, g2o::VertexSE2 vertex_se2);
		
		Eigen::Isometry2d Affine3d2Isometry2d(const Eigen::Affine3d& affine);
		
		
	};

	
	
	inline void G2OGraphOptimization::NDTFeatureLink2EdgeSE2(const NDTFeatureLink& link, g2o::EdgeSE2& edge)
	{
		size_t from = link.getRefIdx() ;
		size_t toward = link.getRefIdx() ;
		
		edge.vertices()[0] = _optimizer.vertex(from);
		edge.vertices()[1] = _optimizer.vertex(toward);
		
		Eigen::Affine3d affine = link.getRelPose();		
		Eigen::Isometry2d isometry2d = Affine3d2Isometry2d(affine);
// 		double x = cumulated_translation(0, 3);
// 		double y = cumulated_translation(1, 3);
		g2o::SE2 se2(isometry2d);
		
		edge.setMeasurement(se2);
		//Information is covariance
		Eigen::Matrix3d cov = link.getRelCov();
		edge.setInformation(cov);

	}
	
	inline void G2OGraphOptimization::NDTFeatureNode2VertexSE2(const NDTFeatureNode& feature, g2o::VertexSE2 vertex_se2)
	{
		Eigen::Affine3d affine = feature.getPose();
// 		getRelPose();		
		Eigen::Isometry2d isometry2d = Affine3d2Isometry2d(affine);
// 		double x = cumulated_translation(0, 3);
// 		double y = cumulated_translation(1, 3);
		g2o::SE2 se2(isometry2d);
		vertex_se2.setEstimate(se2);
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