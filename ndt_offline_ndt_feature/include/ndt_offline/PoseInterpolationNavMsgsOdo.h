#ifndef POSE_INTERPOLATION_NAV_MSG_ODO
#define POSE_INTERPOLATION_NAV_MSG_ODO


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <tf/message_filter.h>
#include <string>
#include <vector>
#include <pcl/conversions.h>
/**
* This makes an offline interpolation for poses using tf::transform
*  This reads the whole file into a vector and after this you can ask poses based on timestamps
*
*/

class PoseInterpolationNavMsgsOdo{
	public:
		/**
		* Constructor
		* @param bagfilename the name of the bagfile
		* @param tftopic the name of the topic for tf like "/tf"
		* @param fixedframe The name of the fixed frame (e.g. "/world")
		* @param maxduration The maximum duration of bag file in secs (the buffer for tftransformer), default 3600
		* @param sensor_link Possibility to add a link, e.g. from base to sensor frame.
		*/ 
		PoseInterpolationNavMsgsOdo(std::string bagfilename, std::string tftopic, 
																std::string _fixedframe, 
																ros::Duration dur = ros::Duration(180), 
																tf::StampedTransform *sensor_link=NULL):
																
																transformer( true, dur)
		{

			bag.open(bagfilename, rosbag::bagmode::Read);
			topics.push_back(tftopic);
			fixedframe = _fixedframe;
			sensor_link_ = sensor_link;
			viewer = new rosbag::View(bag, rosbag::TopicQuery(topics));
			I = viewer->begin();
			first_read_tf = ros::Time(0);
			last_read_tf = ros::Time(0);
			std::cout << "FixedFrame " << fixedframe << " sensor " << sensor_link << " tf topic " << tftopic << std::endl; 
			exit(0);
		}
		
		
		/**
		* Constructor
		* @param bagfilename the name of the bagfile
		* @param tftopic the name of the topic for tf like "/tf"
		* @param fixedframe The name of the fixed frame (e.g. "/world")
		* @param maxduration The maximum duration of bag file in secs (the buffer for tftransformer), default 3600
		* @param sensor_link Possibility to add a link, e.g. from base to sensor frame.
		*/ 
		PoseInterpolationNavMsgsOdo(rosbag::View *view,
																std::string tftopic, 
																std::string _fixedframe, 
																ros::Duration dur = ros::Duration(180), 
																tf::StampedTransform *sensor_link=NULL):																
																transformer( true, dur)
		{
			
			topics.push_back(tftopic);
			fixedframe = _fixedframe;
			sensor_link_ = sensor_link;
			viewer = view;
			I = viewer->begin();
			std::cout << "FixedFrame " << fixedframe << " sensor " << sensor_link << " tf topic " << tftopic << std::endl; 
// 			exit(0);
		}
		
		~PoseInterpolationNavMsgsOdo(){
			
		}
		
		
		/**
		* Reads everything to cache (SLOW to use)
		*/
		void readBagFile(tf::StampedTransform *sensor_link=NULL);
		
		/**
		* Reads up until given time
		*/
		void readUntilTime(ros::Time t);
		
		/**
		* Returns the interpolated Affine transformation for a Time t1
		* @param t0 The time that acts as a reference for computing differential motion
		* @param t1 The time (after t0) that is estimated for T
		* @param frame_id The id of the frame that you want to use (e.g. "/odom")
		* @param &T Output as Eigen::Affine3d 
		*/
		bool getTransformationForTime(ros::Time t0,ros::Time t1, std::string frame_id,Eigen::Affine3d &T);
		bool getTransformationForTime(ros::Time t0,ros::Time t1, std::string frame_id, std::string fixed_frame_id, Eigen::Affine3d &T);
		
		/**
		* Returns the global pose for the t0
		*/		
		bool getTransformationForTime(ros::Time t0, std::string frame_id,tf::Transform &T);
		bool getTransformationForTime(ros::Time t0, std::string frame_id, std::string fixed_frame_id, tf::Transform &T);
		/**
		* Returns the interpolated tf transformation for a Time t1
		* @param t0 The time that acts as a reference for computing differential motion
		* @param t1 The time (after t0) that is estimated for T
		* @param frame_id The id of the frame that you want to use (e.g. "/odom")
		* @param &T Output as tf::Transform 
		*/
		bool getTransformationForTime(ros::Time t0,ros::Time t1, std::string frame_id,tf::Transform &T);
		bool getTransformationForTime(ros::Time t0,ros::Time t1, std::string frame_id, std::string fixed_frame_id, tf::Transform &T);

		bool getTransformationForTime(ros::Time t0, std::string frame_id,Eigen::Affine3d &T);
		

		ros::Time getLastReadTime() const {
		  return last_read_tf;
		}
		ros::Time getFirstReadTime() const {
		  return first_read_tf;
		}
	private:
		rosbag::Bag bag;
		std::vector<std::string> topics;
		tf::Transformer transformer;
		std::string fixedframe;
		tf::StampedTransform *sensor_link_;
		rosbag::View *viewer; 
		rosbag::View::iterator I; 
		void TransformTFToEigen(const tf::Transform &t, Eigen::Affine3d &k);
		ros::Time last_read_tf;
		ros::Time first_read_tf;
};

/**
* This reads all in once
*/
void PoseInterpolationNavMsgsOdo::readBagFile(tf::StampedTransform *sensor_link){
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	
		BOOST_FOREACH(rosbag::MessageInstance const m, view)
		{
			//////////////////////////////////////////////////////////////////////////////////
			tf::tfMessage::ConstPtr transform = m.instantiate<tf::tfMessage>();
			if (transform != NULL){
				if(m.getTopic() == topics[0]){
					for (int i = 0; i < transform->transforms.size(); i++){
							std::cout << "frame_id : " << transform->transforms[i].header.frame_id;
							std::cout << " cframe_id : " << transform->transforms[i].child_frame_id << std::endl;
							tf::StampedTransform stf;
							transformStampedMsgToTF(transform->transforms[i], stf);
							transformer.setTransform(stf);
					}
					if(sensor_link != NULL){
							sensor_link->stamp_ = transform->transforms[0].header.stamp;
							fprintf(stderr,"Setting sensor link %s -> %s\n", sensor_link->frame_id_.c_str(), sensor_link->child_frame_id_.c_str());
							transformer.setTransform(*sensor_link);
						}else{
							//fprintf(stderr,"NULL\n");
						}
						last_read_tf = transform->transforms[0].header.stamp;
						if (first_read_tf == ros::Time(0))
						  first_read_tf = last_read_tf;
				}
			}
			/////////////////////////////////////////////////////////////////////////////////////////////
		}
		std::cout << " [first, last] entry : [" << first_read_tf << ", " << last_read_tf << "]" << std::endl;
		fprintf(stderr,"Cache length %lf",transformer.getCacheLength().toSec());
		bag.close();
		exit(0);
		
}

void PoseInterpolationNavMsgsOdo::readUntilTime(ros::Time t){
    if(I == viewer->end()){
	fprintf(stderr,"END:");
    }
    if(last_read_tf>t){
// 	fprintf(stderr,"No need to read!\n");
	return; ///No need to read new ones
    }

    while(last_read_tf <= t && I != viewer->end()){
		rosbag::MessageInstance const m = *I;
// 		fprintf(stderr,"READING\n");
		//////////////////////////////////////////////////////////////////////////////////
		tf::tfMessage::ConstPtr transform = m.instantiate<tf::tfMessage>();
		if (transform != NULL){
			if(m.getTopic() == topics[0]){

				if(transform->transforms.size()<=0) continue;

				for (int i = 0; i < transform->transforms.size(); i++){
// 					std::cout << "frame_id : " << transform->transforms[i].header.frame_id;
// 					std::cout << " cframe_id : " << transform->transforms[i].child_frame_id << std::endl;
					tf::StampedTransform stf;
					transformStampedMsgToTF(transform->transforms[i], stf);
					double x = stf.getOrigin().x();
					double y = stf.getOrigin().y();
					double z = stf.getOrigin().z();
					double roll, pitch, yaw;
					stf.getBasis().getRPY(roll, pitch, yaw);
// 					std::cout << " adding between " << transform->transforms[i].header.frame_id <<  "and " << transform->transforms[i].child_frame_id << " at time " << stf.stamp_ << " we have " << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << std::endl;
					transformer.setTransform(stf);
				}

				last_read_tf = transform->transforms[0].header.stamp;
				if (first_read_tf == ros::Time(0))
				first_read_tf = last_read_tf;

				if(sensor_link_ != NULL){
					sensor_link_->stamp_ = transform->transforms[0].header.stamp;
// 					fprintf(stderr,"Setting sensor link %s -> %s\n", sensor_link_->frame_id_.c_str(), sensor_link_->child_frame_id_.c_str());
					transformer.setTransform(*sensor_link_);
				}else{
// 					fprintf(stderr,"NULL\n");
				}
			}
		}else{
// 			std::cout << "NULL transfo" << std::endl;
		}
		I++;
		/////////////////////////////////////////////////////////////////////////////////////////////
    }
   
//     fprintf(stderr,"Cache length %lf",transformer.getCacheLength().toSec());
    //bag.close();

}




void PoseInterpolationNavMsgsOdo::TransformTFToEigen(const tf::Transform &t, Eigen::Affine3d &k)
 {
     for(int i=0; i<3; i++)
     {
       k.matrix()(i,3) = t.getOrigin()[i];
       for(int j=0; j<3; j++)
       {
         k.matrix()(i,j) = t.getBasis()[i][j];
       }
     }
     // Fill in identity in last row
     for (int col = 0 ; col < 3; col ++)
       k.matrix()(3, col) = 0;
     k.matrix()(3,3) = 1;
 
}
 

/**
* Returns the interpolated Affine transformation for a Time t
*/
bool PoseInterpolationNavMsgsOdo::getTransformationForTime(ros::Time t0,ros::Time t1, std::string frame_id,tf::Transform &T){
	getTransformationForTime(t0, t1, frame_id, fixedframe, T);
}

bool PoseInterpolationNavMsgsOdo::getTransformationForTime(ros::Time t0,ros::Time t1, std::string frame_id, std::string fixed_frame_id, tf::Transform &T){
	
	std::cout << "Transfo toward base" << frame_id << " " <<  fixed_frame_id << std::endl;
	
	tf::StampedTransform transform;
	//fprintf(stderr,"DT: %lf ",t0.toSec()-t1.toSec());
	
	if(t1 > t0) readUntilTime(t1);
	else readUntilTime(t0);
	
	std::string schaiba;
	
	if(!transformer.canTransform 	(frame_id, t0, frame_id, t1, fixed_frame_id, &schaiba)){
		fprintf(stderr,"FAIL\n");
		return false;
	}
	else{
		std::cout << "Return true" << std::endl;
	}

	//transformer.lookupTransform(frame_id,t0, frame_id, t1, fixedframe, transform);
	transformer.lookupTransform(frame_id, t0, frame_id, t1, fixed_frame_id, transform);
	T = transform;
	
	/* Eigen::Affine3d Ta; */
	/* TransformTFToEigen (transform, Ta); */
	/* std::cout<<"trans: "<<Ta.translation().transpose(); */
	/* std::cout<<":: euler: "<<Ta.rotation().eulerAngles(0,1,2).transpose()<<std::endl; */
	return true;
}


/**
* Returns the interpolated Affine transformation for a Time t
*/
bool PoseInterpolationNavMsgsOdo::getTransformationForTime(ros::Time t0,ros::Time t1, std::string frame_id, Eigen::Affine3d &T){
	getTransformationForTime(t0, t1, frame_id, fixedframe, T);
}

/**
* Returns the interpolated Affine transformation for a Time t
*/
bool PoseInterpolationNavMsgsOdo::getTransformationForTime(ros::Time t0,ros::Time t1, std::string frame_id, std::string fixed_frame_id, Eigen::Affine3d &T){

// 	std::cout << "Transfo toward base" << std::endl;
	tf::StampedTransform transform;
	bool ret_val = PoseInterpolationNavMsgsOdo::getTransformationForTime(t0, t1, frame_id, fixed_frame_id, transform);
	TransformTFToEigen (transform, T);
	return ret_val;
}


/**
* Returns the sensor pose for time t
*/
bool PoseInterpolationNavMsgsOdo::getTransformationForTime(ros::Time t0, std::string frame_id,tf::Transform &T){
	getTransformationForTime(t0, frame_id, fixedframe, T);
}

/**
* Returns the sensor pose for time t
*/
bool PoseInterpolationNavMsgsOdo::getTransformationForTime(ros::Time t0, std::string frame_id, std::string fixed_frame_id, tf::Transform &T){
	
	std::cout << " here Transfo 1.toward 2.base: " << frame_id << " " <<  fixed_frame_id << std::endl;
	
	tf::StampedTransform transform;
// 	fprintf(stderr,"DT: %lf ", (t0 - t0+ros::Duration(1.0)).toSec());
// 	fprintf(stderr,"from to: %lf, %lf \n",t0.toSec(), (t0+ros::Duration(1.0)).toSec());
	std::string schaiba;
	readUntilTime(t0+ros::Duration(1.0));
// 	printf("looking up tf from %s to %s\n",frame_id.c_str(),fixedframe.c_str());
	if(!transformer.canTransform(fixed_frame_id , frame_id, t0, &schaiba)){
		std::cout << "Not found " << std::endl;
		exit(0);
		return false;
	}
	else{
		std::cout<< "Is ok " << std::endl;
	}
// 	printf("found\n");
	transformer.lookupTransform(fixed_frame_id,frame_id,t0, transform);
	T = transform;
	
	double x = transform.getOrigin().x();
	double y = transform.getOrigin().y();
	double z = transform.getOrigin().z();
	double roll, pitch, yaw;
	transform.getBasis().getRPY(roll, pitch, yaw);
// 	std::cout << " between " << fixed_frame_id <<  " and " << frame_id << " at time " << t0 << " we have " << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << " error " << schaiba << std::endl;
	
// 	exit(0);
	
	return true;
}


bool PoseInterpolationNavMsgsOdo::getTransformationForTime(ros::Time t0, std::string frame_id, Eigen::Affine3d &T){
	tf::StampedTransform transform;
	bool ret_val = PoseInterpolationNavMsgsOdo::getTransformationForTime(t0, frame_id, transform);
	TransformTFToEigen (transform, T);
	return ret_val;
}

#endif
