#ifndef PERCEPTIONORU_BASEREADERINTERFACE_08052017
#define PERCEPTIONORU_BASEREADERINTERFACE_08052017
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <laser_geometry/laser_geometry.h>
// #include <velodyne_pointcloud/rawdata.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
// #include <velodyne_pointcloud/point_types.h>
#include <ndt_offline/PoseInterpolationNavMsgsOdo.h>
#include <iostream>
#include <pcl_ros/impl/transforms.hpp>
#include <tf_conversions/tf_eigen.h>


    #include <boost/foreach.hpp>
    #define foreach BOOST_FOREACH

#ifdef READ_RMLD_MESSAGES
#include<SynchronizedRMLD.h>
#endif

namespace perception_oru{
	namespace ndt_offline{
		template<typename PointT>
		class BagReaderInterface{
			
		protected:
			PoseInterpolationNavMsgsOdo *odosync;
			rosbag::Bag bag;
			rosbag::View *view;
			rosbag::View::iterator I;
			std::string velodynetopic_;
			std::string tf_base_link_;
			std::string tf_sensor_link_;
			std::string fixed_frame_id_;
			sensor_msgs::LaserScan::ConstPtr global_scan;
			ros::Time timestamp_of_last_sensor_message;
			Eigen::Affine3d _last_pose;
			///Pose of the sensor with respect to the world frame
			Eigen::Affine3d _baselink_pose;
			///Pose of the sensor with respect to the robot base
			Eigen::Affine3d _sensor_pose;
			Eigen::Affine3d motion_;
			ros::Duration sensor_time_offset_;
			
			
		public:
			/**
			* Constructor
			* @param calibration_file path and name to your velodyne calibration file
			* @param bagfilename The path and name of the bagfile you want to handle
			* @param velodynetopic The topic that contains velodyne_msgs/VelodyneScan
			* @param tf_pose_id The id of the tf that you want to use
			* @param fixed_frame_id The name of the fixed frame in tf (default = "/world")
			* @param tftopic name of tf (default "/tf")
			* @param dur The buffer size (must be larger than the length of the bag file gives) default = 3600s
			* @param sensor_link An optional static link that takes e.g. your /odom to the sensor frame 
			*/
			BagReaderInterface(std::string calibration_file, 
				std::string bagfilename,
				std::string velodynetopic, 
				std::string tf_base_link, 
				std::string tf_sensor_link,
				std::string fixed_frame_id="/world",
				std::string tftopic="/tf", 
				ros::Duration dur = ros::Duration(3600),
				tf::StampedTransform *sensor_link=NULL,
				double velodyne_max_range=130.0, 
				double velodyne_min_range=2.0,
				double sensor_time_offset=0.0) : fixed_frame_id_(fixed_frame_id), velodynetopic_(velodynetopic), tf_base_link_(tf_base_link), tf_sensor_link_(tf_sensor_link)
			
			{
// 				dataParser.setupOffline(calibration_file, velodyne_max_range, velodyne_min_range); 
				sensor_time_offset_ = ros::Duration(sensor_time_offset);
				fprintf(stderr,"Opening '%s'\n",bagfilename.c_str());

				bag.open(bagfilename, rosbag::bagmode::Read);

				

				std::vector<std::string> topics;
				topics.push_back(tftopic);
				topics.push_back(velodynetopic_);
		#ifdef READ_RMLD_MESSAGES
				topics.push_back("/rmld/data");
				topics.push_back("/amtec/tilt_state");
		#endif
				for(int i=0; i<topics.size(); ++i) { 
				fprintf(stderr,"Searched Topic [%d] = '%s'\n",i,topics[i].c_str());
				}
// 				exit(0);

				view = new rosbag::View(bag, rosbag::TopicQuery(topics));
				I = view->begin();
				
				assert(I != view->end());

				odosync = new PoseInterpolationNavMsgsOdo(view, tftopic, fixed_frame_id, dur, sensor_link);
		#ifdef READ_RMLD_MESSAGES
				rmldsync = new SynchronizedRMLD(view,tftopic,"/base_link",dur);
		#endif
				
				assert(I != view->end());
				rosbag::MessageInstance const m = *(this->I);
// 					global_scan = m.instantiate<velodyne_msgs::VelodyneScan>();
				this->global_scan = m.instantiate<sensor_msgs::LaserScan>();
				this->I++;
				
				while(this->global_scan == NULL){
					std::cout << "forward " << std::endl;
					rosbag::MessageInstance const m_tmp = *(this->I);
					this->global_scan = m_tmp.instantiate<sensor_msgs::LaserScan>();
					this->I++;
				}
								
				timestamp_of_last_sensor_message = this->global_scan->header.stamp;
					
				//Update motion
				//Get BaseLink Pose
				tf::Transform pose_bl;
				if(!getPoseFor( this->tf_base_link_, this->fixed_frame_id_, pose_bl)){
					std::cout << "Not transform found between " << this->tf_base_link_ << " and " << this->fixed_frame_id_ << std::endl;
					throw std::runtime_error("Not transform found");
				}
// 				tf::transformTFToEigen (pose_bl, _baselink_pose);
				transformtoAffine2d(pose_bl, _baselink_pose);

				_last_pose = _baselink_pose;
				
				std::cout << "Last pose " << std::endl << _baselink_pose.matrix() << std::endl;
				
				//Get sensor Pose
				tf::Transform pose_sensor;
				if(!getPoseFor(this->tf_sensor_link_, this->tf_base_link_, pose_sensor)){
					std::cout << "Not transform found between " << this->tf_sensor_link_ << " and " << this->tf_base_link_ << std::endl;
					throw std::runtime_error("Not transform found");
				}
				
				double x = pose_sensor.getOrigin().x();
				double y = pose_sensor.getOrigin().y();
				double z = pose_sensor.getOrigin().z();
				double roll, pitch, yaw;
				pose_sensor.getBasis().getRPY(roll, pitch, yaw);
				std::cout << " between so: "<< x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << std::endl;
	
	
// 				tf::transformTFToEigen (pose_sensor, _sensor_pose);
				transformtoAffine2d(pose_sensor, _sensor_pose);
				std::cout << "sensor pose " << std::endl  << _sensor_pose.matrix() << std::endl;
				
// 				exit(0);
				std::cout << "GOT " << m.getTopic() << " at " << this->global_scan->header.stamp << std::endl;


				std::cout << "end initi" << std::endl;
// 				exit(0);
				
			}
			
			void print(){
				std::cout <<
				"velodyne topic: " << velodynetopic_ << "\n" <<
				"tf_base_link: " << tf_base_link_ <<"\n" <<
				"tf sensor link: " << tf_sensor_link_<<"\n" <<
				"fixed frame: " << fixed_frame_id_<< std::endl;
			}
			
			Eigen::Affine3d& getMotion(){return motion_;}
			const Eigen::Affine3d& getMotion() const {return motion_;}
			Eigen::Affine3d& getLastPose(){return _last_pose;}
			const Eigen::Affine3d& getLastPose() const {return _last_pose;}
			Eigen::Affine3d& getBaseLinkPose(){return _baselink_pose;}
			Eigen::Affine3d& getSensorPose(){return _sensor_pose;}
			sensor_msgs::LaserScan::ConstPtr getLastLaserScan(){return global_scan;}
			const sensor_msgs::LaserScan::ConstPtr& getLastLaserScan() const {return global_scan;}
			
			/**
			* Get pose for latest measurement with pose id
			*/
			bool getPoseFor(std::string toward, std::string base, tf::Transform &pose){
				if(odosync->getTransformationForTime(timestamp_of_last_sensor_message, toward, base, pose)){
					return true;
				}
				return false;
			}
			
			ros::Time getTimeStampOfLastSensorMsg() const {
                 return timestamp_of_last_sensor_message;
            }
            
            void transformtoAffine2d(const tf::Transform& transform, Eigen::Affine3d& pose){
				double x = transform.getOrigin().x();
				double y = transform.getOrigin().y();
				double z = 0;
				double roll, pitch, yaw;
				transform.getBasis().getRPY(roll, pitch, yaw);
			
				pose = Eigen::Translation<double,3>(x,y,z)*
				Eigen::AngleAxis<double>(roll,Eigen::Vector3d::UnitX()) *
				Eigen::AngleAxis<double>(pitch,Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxis<double>(yaw,Eigen::Vector3d::UnitZ()) ;
			}
            
            /**
			 * @brief Read one scan and update last time stamp, the transformation between the last pose and this one
			 */
            bool getNextScanMsg(){
				if(this->I == this->view->end()){
					fprintf(stderr,"End of measurement file\n");
					return false;
				}
				bool done = false; 
				while(!done){
					rosbag::MessageInstance const m = *(this->I);
// 					global_scan = m.instantiate<velodyne_msgs::VelodyneScan>();
					this->global_scan = m.instantiate<sensor_msgs::LaserScan>();
					this->I++;
					if(this->I == this->view->end()) done = true;
					if(this->global_scan != NULL){
						
						timestamp_of_last_sensor_message = this->global_scan->header.stamp;
						
						//Update motion
						//Get BaseLink Pose
						tf::Transform pose_bl;
						getPoseFor( this->tf_base_link_, this->fixed_frame_id_, pose_bl);
// 						tf::transformTFToEigen (pose_bl, _baselink_pose);
						transformtoAffine2d(pose_bl, _baselink_pose);
						
// 						std::cout << "last pose " <<_last_pose.matrix() << std::endl << std::endl << "base link  " << _baselink_pose.matrix() << std::endl << std::endl;
						
						motion_ = _last_pose.inverse() * _baselink_pose;
						_last_pose = _baselink_pose;
						
// 						std::cout << "motion " << motion_.matrix() << std::endl << std::endl;
						
						//Get sensor Pose
						tf::Transform pose_sensor;
						getPoseFor(this->tf_sensor_link_, this->tf_base_link_, pose_sensor);
// 						tf::transformTFToEigen (pose_sensor, _sensor_pose);
						transformtoAffine2d(pose_sensor, _sensor_pose);
						
// 						std::cout << "GOT " << m.getTopic() << " at " << this->global_scan->header.stamp << " on frame " << this->global_scan->header.frame_id << std::endl;
						done = true;
					}
				}
				if(this->I == this->view->end()) {
					return false;
				}
				return true;
			}
			
			
			
		};
	}
}

#endif