#pragma once

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/OccupancyGrid.h>

#include "ndt_feature_graph.h"

inline void moveOccupancyMap(nav_msgs::OccupancyGrid &occ_grid, const Eigen::Affine3d &pose) {

  Eigen::Affine3d map_origin;
  tf::poseMsgToEigen(occ_grid.info.origin, map_origin);
  Eigen::Affine3d new_map_origin = pose*map_origin;
  tf::poseEigenToMsg(new_map_origin, occ_grid.info.origin);
}

namespace ndt_feature{

	inline void initSensorPose2D(NDTFeatureGraph& graph, const std::string& robot_frame, const std::string& sensor_frame){
		tf::TransformListener listener;
		tf::StampedTransform transform;
		try {
			listener.waitForTransform(sensor_frame, robot_frame, ros::Time(0), ros::Duration(10.0) );
			listener.lookupTransform(sensor_frame, robot_frame, ros::Time(0), transform);
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s",ex.what());
// 			exit(0);
		}
		double x = transform.getOrigin().x();
		double y = transform.getOrigin().y();
		double z = 0;
		double roll, pitch, yaw;
		transform.getBasis().getRPY(roll, pitch, yaw);
		
		Eigen::Affine3d pose_sensor = Eigen::Translation<double,3>(x,y,z)*
		Eigen::AngleAxis<double>(roll,Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxis<double>(pitch,Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxis<double>(yaw,Eigen::Vector3d::UnitZ()) ;
		
		std::cout << "Sensor pose " << pose_sensor.matrix() << std::endl;
// 		exit(0);
		
		graph.setSensorPose(pose_sensor);
		
	}
		
		
	inline void initRobotPose2D(NDTFeatureGraph& graph, pcl::PointCloud< pcl::PointXYZ >& cloud, std::string& world_frame, std::string& robot_frame, const InterestPointVec& pts, bool preLoad=false)
	{
		std::cout << "Listening between " << world_frame << " and " << robot_frame << std::endl;
		tf::TransformListener listener;
		tf::StampedTransform transform;
		try {
			std::cout << "wait" << std::endl;
			listener.waitForTransform(robot_frame, world_frame, ros::Time(0), ros::Duration(1.0) );
		std::cout << "Lookup" << std::endl;
			listener.lookupTransform(world_frame, robot_frame, ros::Time(0), transform);
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s",ex.what());
		}
		std::cout << "DONE " << robot_frame << std::endl;
		double x = transform.getOrigin().x();
		double y = transform.getOrigin().y();
		double z = 0;
		double roll, pitch, yaw;
		transform.getBasis().getRPY(roll, pitch, yaw);
		
		Eigen::Affine3d pose = Eigen::Translation<double,3>(x,y,z)*
		Eigen::AngleAxis<double>(roll,Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxis<double>(pitch,Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxis<double>(yaw,Eigen::Vector3d::UnitZ()) ;
		std::cout << "POSE " << pose.matrix() << std::endl;
// 			exit(0);
		
		graph.initialize(pose, cloud, pts, preLoad);

	}

}