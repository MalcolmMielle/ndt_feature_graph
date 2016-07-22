#ifndef NDTFEATURE_HARCODED_OLD_22072016
#define NDTFEATURE_HARCODED_OLD_22072016

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

// #include "ndt_feature/ndt_feature_graph.h"
// #include "ndt_feature/utils.h"

#include "Eigen/Core"

namespace ndt_feature {
	
	class GraphG2OHardCoded{
	public :
		typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
		typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
	protected:
		g2o::SparseOptimizer _optimizer;
		SlamLinearSolver* _linearSolver;
		SlamBlockSolver* _blockSolver;
		g2o::OptimizationAlgorithmGaussNewton* _solver;
		g2o::SE2 _sensorOffsetTransf;
		g2o::ParameterSE2Offset* _sensorOffset;
		Eigen::IOFormat cleanFmt;
		
		std::vector<Eigen::Vector3d> _robot_positions;
		std::vector<Eigen::Vector2d> _landmark_positions;
		std::vector<Eigen::Vector2d> _prior_landmark_positions;
		std::vector<std::tuple<g2o::SE2, int, int> > _odometry;
		std::vector<std::tuple<Eigen::Vector2d, int, int> > _observation_real_landmarks;
		std::vector<std::tuple<Eigen::Vector2d, int, int> > _edges_prior;
		std::vector<std::tuple<Eigen::Vector2d, int, int> > _link_in_between_maps;
		
		Eigen::Vector2d _transNoise;
		double _rotNoise;
		Eigen::Vector2d _landmarkNoise;
		Eigen::Vector2d _priorNoise;
		Eigen::Vector2d _linkNoise;
		
	public:
		// :( I hate pointers
		GraphG2OHardCoded() : _sensorOffsetTransf(0.2, 0.1, -0.1), cleanFmt(4, 0, ", ", "\n", "[", "]"), _transNoise(0.05, 0.01), _rotNoise(DEG2RAD(2.)), _landmarkNoise(0.05, 0.05), _priorNoise(0.05, 0.01), _linkNoise(0.2, 0.2){
			
			_linearSolver = new SlamLinearSolver();
			_linearSolver->setBlockOrdering(false);
			_blockSolver = new SlamBlockSolver(_linearSolver);
			_solver = new g2o::OptimizationAlgorithmGaussNewton(_blockSolver);
			_linearSolver->setBlockOrdering(false);
			_optimizer.setAlgorithm(_solver);

			// add the parameter representing the sensor offset ATTENTION was ist das ?
			_sensorOffset = new g2o::ParameterSE2Offset;
			_sensorOffset->setOffset(_sensorOffsetTransf);
			_sensorOffset->setId(0);
			_optimizer.addParameter(_sensorOffset);
			
			//First robot position
			//x
			Eigen::Vector3d robot1;
			robot1 << 0, 0, 0;
			_robot_positions.push_back(robot1);
			
			//Second robot position
			Eigen::Vector3d robot2;
			robot2 << 2, 0, 0;
			_robot_positions.push_back(robot2);
			
			//Third robot position
			Eigen::Vector3d robot3;
			robot3 << 3, -1, 0;
			_robot_positions.push_back(robot3);
			
			//Real landmark position
			Eigen::Vector2d lan1;
			lan1 << 2, 1;
			_landmark_positions.push_back(lan1);
			Eigen::Vector2d lan2;
			lan2 << 1, -2;
			_landmark_positions.push_back(lan2);
			
			//Prior landmark positio
			Eigen::Vector2d prior_lan1;
			prior_lan1 << 1, -2;
			_prior_landmark_positions.push_back(prior_lan1);
			
			Eigen::Vector2d prior_lan2;
			prior_lan2 << 6, -2;
			_prior_landmark_positions.push_back(prior_lan2);
			
			Eigen::Vector2d prior_lan3;
			prior_lan3 << 9, 2;
			_prior_landmark_positions.push_back(prior_lan3);
			
			Eigen::Vector2d prior_lan4;
			prior_lan4 << 2, 1;
			_prior_landmark_positions.push_back(prior_lan4);
			
			//Odometry : SE2 == transformation
			g2o::SE2 od1(2.5, 0, 0);
			std::tuple<g2o::SE2, int, int> odom1(od1, 0, 1);
			g2o::SE2 od2(1.5, -1.5, 20);
			std::tuple<g2o::SE2, int, int> odom2(od2, 1, 2);
			_odometry.push_back(odom1);
			_odometry.push_back(odom2);
			
			//Observation real landmarks
			Eigen::Vector2d observation1;
			observation1 << 1, 1;
			std::tuple<Eigen::Vector2d, int, int> obs1(observation1, 0, 3);
			std::tuple<Eigen::Vector2d, int, int> obs2(observation1, 1, 3);
			_observation_real_landmarks.push_back(obs1);
			_observation_real_landmarks.push_back(obs2);
			
			Eigen::Vector2d observation3;
			observation3 << 1, -5;
			std::tuple<Eigen::Vector2d, int, int> obs3(observation3, 0, 4);
			_observation_real_landmarks.push_back(obs3);
			
			Eigen::Vector2d observation4;
			observation4 << -1, -3;
			std::tuple<Eigen::Vector2d, int, int> obs4(observation4, 1, 4);
			_observation_real_landmarks.push_back(obs4);
			
			Eigen::Vector2d observation5;
			observation5 << -2, -1;
			std::tuple<Eigen::Vector2d, int, int> obs5(observation5, 2, 4);
			_observation_real_landmarks.push_back(obs5);
			
			//Edge from the prior map
			Eigen::Vector2d edge1;
			edge1 << 5, 0;
			std::tuple<Eigen::Vector2d, int, int> ed1(edge1, 5, 6);
			_edges_prior.push_back(ed1);
			
			Eigen::Vector2d edge2;
			edge2 << 3, 4;
			std::tuple<Eigen::Vector2d, int, int> ed2(edge2, 6, 7);
			_edges_prior.push_back(ed2);
			
			Eigen::Vector2d edge3;
			edge3 << -7, -1;
			std::tuple<Eigen::Vector2d, int, int> ed3(edge3, 7, 8);
			_edges_prior.push_back(ed3);
			
			//Prior to landmark
			Eigen::Vector2d link1;
			link1 << 0, 0;
			std::tuple<Eigen::Vector2d, int, int> lnk1(link1, 4, 5);
			_link_in_between_maps.push_back(lnk1);
			
			std::tuple<Eigen::Vector2d, int, int> lnk2(link1, 3, 8);
			_link_in_between_maps.push_back(lnk2);
			
			
		};
		
		
		void save(const std::string& file){
			_optimizer.save(file.c_str());
		}
		
		void makeGraph(){
			
			int id = 0;
			
			/******************Adding robot nodes*************/
			
			for (size_t i = 0; i < _robot_positions.size(); ++i) {
				std::cout << "Adding robot node " << i/3 << std::endl;
				g2o::VertexSE2* robot =  new g2o::VertexSE2;
				robot->setId(id);
				++id;
				Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "\t", " ", "", "", "", "");
// 				std::cout <<"Estimate " << isometry2d.matrix().format(cleanFmt) << std::endl;
				g2o::SE2 se2(_robot_positions[i](0), _robot_positions[i](1), _robot_positions[i](2));
				
				std::cout << "SE2 " << se2.toVector() << std::endl;
				robot->setEstimate(se2);
				std::cout << "Robot " << robot->estimate().toVector() << std::endl;
				_optimizer.addVertex(robot);	
			}
			
			Eigen::Matrix3d covariance;
			covariance.fill(0.);
			covariance(0, 0) = _transNoise[0]*_transNoise[0];
			covariance(1, 1) = _transNoise[1]*_transNoise[1];
			covariance(2, 2) = _rotNoise*_rotNoise;
			Eigen::Matrix3d information = covariance.inverse();
			
			//Adding odometry
			for(size_t i = 0 ; i < _odometry.size() ; ++i){
				
				g2o::EdgeSE2* odometry = new g2o::EdgeSE2;
				odometry->vertices()[0] = _optimizer.vertex(std::get<1>(_odometry[i])) ;
				odometry->vertices()[1] = _optimizer.vertex(std::get<2>(_odometry[i])) ;
				odometry->setMeasurement(std::get<0>(_odometry[i]) /*simEdge.simulatorTransf*/);
				odometry->setInformation(information);
				_optimizer.addEdge(odometry);
				
				
			}
			
			/****************** Adding Landmarks nodes *************/
			
			Eigen::Matrix2d covariance_landmark; 
			covariance_landmark.fill(0.);
			covariance_landmark(0, 0) = _landmarkNoise[0]*_landmarkNoise[0];
			covariance_landmark(1, 1) = _landmarkNoise[1]*_landmarkNoise[1];
			Eigen::Matrix2d information_landmark = covariance_landmark.inverse();
			
			std::cerr << "Optimization: add landmark vertices ... ";
			for (size_t i = 0; i < _landmark_positions.size() ; ++i) {
				g2o::VertexPointXY* landmark = new g2o::VertexPointXY;
				landmark->setId(id);
				++id;
// 				g2o::SE2 se2(_landmark_positions[i], _landmark_positions[i+1], _landmark_positions[i+2]);
				landmark->setEstimate(_landmark_positions[i]);
				_optimizer.addVertex(landmark);
			}
			std::cerr << "done." << std::endl;

			std::cerr << "Optimization: add landmark observations ... ";
			for (size_t i = 0; i < _observation_real_landmarks.size(); ++i) {
				g2o::EdgeSE2PointXY* landmarkObservation =  new g2o::EdgeSE2PointXY;
				landmarkObservation->vertices()[0] = _optimizer.vertex(std::get<1>(_observation_real_landmarks[i]) );
				landmarkObservation->vertices()[1] = _optimizer.vertex(std::get<2>(_observation_real_landmarks[i]));
				landmarkObservation->setMeasurement(std::get<0>(_observation_real_landmarks[i]));
				landmarkObservation->setInformation(information_landmark);
				//TODO
				landmarkObservation->setParameterId(0, _sensorOffset->id());
				_optimizer.addEdge(landmarkObservation);
			}
			std::cerr << "done." << std::endl;
			
			/****************** Adding Prior nodes *************/
			
			Eigen::Matrix2d covariance_prior; 
			covariance_prior.fill(0.);
			covariance_prior(0, 0) = _priorNoise[0]*_priorNoise[0];
			covariance_prior(1, 1) = _priorNoise[1]*_priorNoise[1];
			Eigen::Matrix2d information_prior = covariance_prior.inverse();
			
			std::cerr << "Optimization: add prior landmark vertices ... ";
			for (size_t i = 0; i < _prior_landmark_positions.size(); ++i) {
				g2o::VertexPointXY* landmark = new g2o::VertexPointXY;
				landmark->setId(id);
				++id;
// 				g2o::SE2 se2(_prior_landmark_positions[i], _prior_landmark_positions[i+1], _prior_landmark_positions[i+2]);
				landmark->setEstimate(_prior_landmark_positions[i]);
				_optimizer.addVertex(landmark);
			}
			std::cerr << "done." << std::endl;
			
			std::cerr << "Optimization: add wall prior ... ";
			for (size_t i = 0; i < _edges_prior.size(); ++i) {
				g2o::EdgeSE2PointXY* landmarkObservation =  new g2o::EdgeSE2PointXY;
				landmarkObservation->vertices()[0] = _optimizer.vertex(std::get<1>(_edges_prior[i]));
				landmarkObservation->vertices()[1] = _optimizer.vertex(std::get<2>(_edges_prior[i]));
				landmarkObservation->setMeasurement(std::get<0>(_edges_prior[i]));
				landmarkObservation->setInformation(information_prior);
				//TODO
				landmarkObservation->setParameterId(0, _sensorOffset->id());
				_optimizer.addEdge(landmarkObservation);
			}
			
			/****************** Adding prior to ndt edges *************/
			
			Eigen::Matrix2d covariance_link; 
			covariance_link.fill(0.);
			covariance_link(0, 0) = _linkNoise[0]*_linkNoise[0];
			covariance_link(1, 1) = _linkNoise[1]*_linkNoise[1];
			Eigen::Matrix2d information_link = covariance_link.inverse();
			
			//Add link between the two maps -> edge oftransform zero
			std::cerr << "Optimization: add wall prior link to ndt ... ";
			for (size_t i = 0; i < _link_in_between_maps.size(); ++i) {
				g2o::EdgeSE2PointXY* landmarkObservation =  new g2o::EdgeSE2PointXY;
				landmarkObservation->vertices()[0] = _optimizer.vertex(std::get<1>(_link_in_between_maps[i]));
				landmarkObservation->vertices()[1] = _optimizer.vertex(std::get<2>(_link_in_between_maps[i]));
				landmarkObservation->setMeasurement(std::get<0>(_link_in_between_maps[i]));
				landmarkObservation->setInformation(information_link);
				//TODO
				landmarkObservation->setParameterId(0, _sensorOffset->id());
				_optimizer.addEdge(landmarkObservation);
			}

			
			std::cerr << "done." << std::endl;
			
			
		}
		
		
		
		
	};
}

#endif