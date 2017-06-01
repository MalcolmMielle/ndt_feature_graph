#ifndef NDT_FUSER_HMT_LOGGER_HH_04052017
#define NDT_FUSER_HMT_LOGGER_HH_04052017

#include <iostream>
#include <fstream>

#include "ros/time.h"

#include "ndt_feature_graph.h"

//#define BASELINE

namespace ndt_feature {
	
	/**
	* \brief This class fuses new point clouds into a common ndt map reference, keeping tack of the 
	* camera postion. It also log the position and orientation of each pose along with a time stamp a file
	* \author Malcolm
	*/
	class NDTFeatureGraphLogger : public NDTFeatureGraph{
		
	protected:
		std::string _file_out_logger;
		int _old_nb_node;
		
	public:
		
		NDTFeatureGraphLogger(const std::string& file_out_logger) : NDTFeatureGraph(), _file_out_logger(file_out_logger), _old_nb_node(0)  {}
  
		NDTFeatureGraphLogger(const std::string& file_out_logger, const NDTFeatureGraph::Params &params, const NDTFeatureFuserHMT::Params &fuserParams) : NDTFeatureGraph(params, fuserParams), _file_out_logger(file_out_logger), _old_nb_node(0) {}
			
		
		/**
		*
		*
		*/
		Eigen::Affine3d update(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud, const InterestPointVec& pts, ros::Time time = ros::Time::now()){
			std::cout << "Update in the logger" << std::endl;
			Eigen::Affine3d Tnow_out = ndt_feature::NDTFeatureGraph::update(Tmotion, cloud, pts);
			//Add the time for every new node since the pose save in the node is the first one
			if(_old_nb_node != nodes_.size()){
				NDTFeatureNode &node = nodes_.back();
				node.time_last_update = time.toSec();
				_old_nb_node = nodes_.size();
			}
			logT(Tnow_out, time);
			return Tnow_out;
			
		}
		
		void logT(const Eigen::Affine3d& T_out, ros::Time time){
			std::cout <<"Log in " << _file_out_logger << std::endl;
			Eigen::Affine2d T2d = eigenAffine3dTo2d(T_out);
			
			Eigen::Rotation2Dd R(0);
			Eigen::Vector2d t(T2d.translation());
			R.fromRotationMatrix(T2d.linear());
			
			std::ofstream out(_file_out_logger.c_str(), std::ios::in | std::ios::out | std::ios::ate);
// 				out.open (_file_out_logger.c_str());
			
			std::cout << t(0) << " " << t(1) << " " << R.angle() << " " << ros::Time::now() << std::endl;
			out << t(0) << " " << t(1) << " " << R.angle() << " " << time << "\n";
			out.close();
			
// 				exit(0);
			
		}
		
	private:
		Eigen::Affine2d eigenAffine3dTo2d(const Eigen::Affine3d &a3d) {
			return Eigen::Translation2d(a3d.translation().topRows<2>()) *
				Eigen::Rotation2D<double>(getRobustYawFromAffine3d(a3d));//a3d.linear().topLeftCorner<2,2>();
		}
		
		double getRobustYawFromAffine3d(const Eigen::Affine3d &a) {
			// To simply get the yaw from the euler angles is super sensitive to numerical errors which will cause roll and pitch to have angles very close to PI...
			Eigen::Vector3d v1(1,0,0);
			Eigen::Vector3d v2 = a.rotation()*v1;
			double dot = v1(0)*v2(0)+v1(1)*v2(1); // Only compute the rotation in xy plane...
			double angle = acos(dot);
			// Need to find the sign
			if (v1(0)*v2(1)-v1(1)*v2(0) > 0)
				return angle;
			return -angle;
		}
		
		
	};

}

#endif