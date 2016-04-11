#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/pointcloud_utils.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <cstdio>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <fstream>

#include <pcl/kdtree/kdtree_flann.h>


Eigen::Affine3d getAsAffine(float x, float y, float yaw ){
	Eigen::Matrix3d m;
	m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
			* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
	Eigen::Translation3d v(x,y,0);
	Eigen::Affine3d T = v*m;
	
	return T;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// SCAN PAIR CLASS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename PointT>
class ScanPair{
	public:	
		pcl::PointCloud<PointT> cloud1;
		Eigen::Affine3d pose1;
		
		pcl::PointCloud<PointT> cloud2;
		Eigen::Affine3d pose2;
		
		ScanPair(pcl::PointCloud<PointT> &_cloud1,Eigen::Affine3d &_pose1, pcl::PointCloud<PointT> &_cloud2, Eigen::Affine3d &_pose2) : 
				cloud1(_cloud1),
				pose1(_pose1),
				cloud2(_cloud2),
				pose2(_pose2)
		{}

		/**
		* Draw the clouds
		*/
		// void drawScans(Eigen::Affine3d &Ts, mrpt::gui::CDisplayWindow3D  &w3D, mrpt::opengl::CPointCloudColouredPtr gl_pnts){
		// 	pcl::PointCloud<PointT> c1=cloud1;
		// 	Eigen::Affine3d p1 = pose1 * Ts;
		// 	lslgeneric::transformPointCloudInPlace(p1, c1);
			
		// 	pcl::PointCloud<PointT> c2=cloud2;
		// 	Eigen::Affine3d p2 = pose2 * Ts;
		// 	lslgeneric::transformPointCloudInPlace(p2, c2);
			
		// 	mrpt::opengl::COpenGLScenePtr &scene = w3D.get3DSceneAndLock();			
		// 	for(int i=0;i<c1.size();i++){
		// 		gl_points->push_back(c1.points[i].x,c1.points[i].y,c1.points[i].z, 1.0,0,0 );
		// 	}
		// 	for(int i=0;i<c2.size();i++){
		// 		gl_points->push_back(c2.points[i].x,c2.points[i].y,c2.points[i].z, 0,1.0,0 );
		// 	}
			
			
		// 	w3D.unlockAccess3DScene();
		// 	w3D.repaint();
		// }
		/**
		 * Compute ICP score for sensor offset
		 */
		double scoreICP(Eigen::Affine3d &Ts){
			pcl::PointCloud<PointT> c1=cloud1;
			Eigen::Affine3d p1 = pose1 * Ts;
			lslgeneric::transformPointCloudInPlace(p1, c1);
			
			pcl::PointCloud<PointT> c2=cloud2;
			Eigen::Affine3d p2 = pose2 * Ts;
			lslgeneric::transformPointCloudInPlace(p2, c2);
			
			pcl::KdTreeFLANN<PointT> kdtree;
			typename pcl::KdTree<PointT>::PointCloudPtr mp (new pcl::PointCloud<PointT>);
			if(c1.size()==0){
                          fprintf(stderr,"What a hell! Num points = %d\n",(int)c1.size());
			}
			//			fprintf(stderr,"1: '%d' -- ",c1.size());
			(*mp) = c1;
			//			fprintf(stderr,"2: '%d' == '%d'\n",c1.size(), mp->size());
			kdtree.setInputCloud (mp);
			int K = 1;
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			double e=0, error_th=0.1*0.1;
			
			for(unsigned int i=0;i<c2.size();i++){
				if ( kdtree.nearestKSearch (c2[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
					if(pointNKNSquaredDistance[0] > error_th){
						e+=error_th;
					}
					else{
						e+= pointNKNSquaredDistance[0];
					}
					//					fprintf(stderr,"%f\n",pointNKNSquaredDistance[0]);
				}
		
			}//FOR
			
			return e;
		}
		
		
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// SCANS THAT ARE USED IN REGISTRATION
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector< ScanPair<pcl::PointXYZ> > pairs;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Methods for optimization (2D-Extrinsic Scanner offset calibration)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Affine3d global_sensor_offset;


double errorFunction(Eigen::Affine3d sensor_offset){
	double e=0;
	for(unsigned int i=0;i<pairs.size();i++){
		e+=pairs[i].scoreICP(sensor_offset);
	}
	return e;
}


// void plotOptimizationResult(Eigen::Affine3d Ts){
// 	mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();
// 	scene->clear();
// 	gl_points->clear();
// 	scene->insert( gl_points );
// 	scene->insert( plane);
// 	win3D.unlockAccess3DScene();
// 	for(unsigned int i=0;i<pairs.size();i++){
// 		pairs[i].drawScans(Ts,win3D,gl_points);
// 	}
// 	win3D.repaint();
// }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Brute Force - perkele!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ix, iy, iway - initial guess x, y, yaw
// x_s, y_s, yaw_w - search region for x will be ix (+/-) x_s, etc.
// t_r, yaw_r - increments used

void bruteForceOptimization(double ix, double iy, double iyaw, double x_s, double y_s, double yaw_s, double t_r, double yaw_r){
	double min_score = 1e6;
	Eigen::Affine3d result_T;
	unsigned int cnt = 0;
	///Create a shit-load-of-poses
	for(double x = ix-x_s;x<ix+x_s;x+=t_r){
		for(double y = iy-y_s;y<iy+y_s;y+=t_r){
			for(double yaw = iyaw-yaw_s;yaw<iyaw+yaw_s;yaw+=yaw_r){
				Eigen::Affine3d tr = getAsAffine(x,y,yaw);
				cnt++;
				if(cnt%1000==0){
					fprintf(stderr,"Progressing at (%lf %lf %lf)\n",x,y,yaw);
					//std::cout<<"Progress: x="<<x<<" y="<<y<<" a="<<yaw;
					//std::cout<<" :: Score:"<<min_score<<" Trans:["<<result_T.translation().transpose()<<"] Yaw:"<<result_T.rotation().eulerAngles(0,1,2)[2]*180.0/M_PI<<std::endl;
				}
				double s = errorFunction(tr);
 				if(s<min_score){
 					min_score=s;
					result_T = tr;
					fprintf(stderr,"New Min=%lf @ ",s);
					fprintf(stderr,"Trans (%lf %lf %lf [deg %lf]) \n",tr.translation().transpose()[0],tr.translation().transpose()[1], tr.rotation().eulerAngles(0,1,2)[2], tr.rotation().eulerAngles(0,1,2)[2]*180.0/M_PI);
					//std::cout<<"Trans:["<<tr.translation().transpose()<<"] Yaw:"<<tr.rotation().eulerAngles(0,1,2)[2]*180.0/M_PI<<std::endl;
					//plotOptimizationResult(tr);
				}
			}
		}
	}
	
}





////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// CALLBACK
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZ> cloud_old;




/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Update measurement
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
///Sensor offset
float offx = 0;
float offy = 0;
float offa = 0;
static bool has_sensor_offset_set = false;
static bool isFirstLoad=true;
Eigen::Affine3d Told,Todo;


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//mrpt::utils::CTicTac	TT;

std::string tf_odo_topic = "odom_base_link";
std::string tf_state_topic = "state_base_link";
std::string tf_world_frame = "world";
ros::Time last_message;
void callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	static int counter = 0;
	counter++;
	last_message = ros::Time::now();
	
	static tf::TransformListener tf_listener;
	//double looptime = TT.Tac();
	//TT.Tic();
	//fprintf(stderr,"Lt( %.1lfms %.1lfHz seq:%d) \n",looptime*1000,1.0/looptime,scan->header.seq);
	double gx,gy,gyaw,x,y,yaw;
	
	///Get state information
	tf::StampedTransform transform;
	tf_listener.waitForTransform(tf_world_frame, tf_state_topic, scan->header.stamp,ros::Duration(1.0));
	try{
		tf_listener.lookupTransform(tf_world_frame, tf_state_topic, scan->header.stamp, transform);
		gyaw = tf::getYaw(transform.getRotation());  
	  gx = transform.getOrigin().x();
	  gy = transform.getOrigin().y();
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}
	try{
		tf_listener.lookupTransform(tf_world_frame, tf_odo_topic, scan->header.stamp, transform);
		yaw = tf::getYaw(transform.getRotation());  
	  x = transform.getOrigin().x();
	  y = transform.getOrigin().y();
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}
	
		
        //	mrpt::utils::CTicTac	tictac;
	//tictac.Tic();

	int N =(scan->angle_max - scan->angle_min)/scan->angle_increment;
	/////
	/// Pose conversions
	////
	
	Eigen::Affine3d T = getAsAffine(x,y,yaw);
	Eigen::Affine3d Tgt = getAsAffine(gx,gy,gyaw);
	
	///Laser scan to PointCloud
	pcl::PointCloud<pcl::PointXYZ> cloud;	
	for (int j=0;j<N;j++){
		double r  = scan->ranges[j];
		if(r>=scan->range_min && r<scan->range_max && r>0.3 && r<20.0){
			double a  = scan->angle_min + j*scan->angle_increment;
			pcl::PointXYZ pt;
			pt.x = r*cos(a);
			pt.y = r*sin(a);
			pt.z = 0.1+(((double)rand())/(double)(RAND_MAX))*0.01;
			cloud.push_back(pt);
		}
	}
	
	if(isFirstLoad){
		Told = Tgt;
		Todo = Tgt;
		cloud_old = cloud;
		isFirstLoad = false;
	}
	Eigen::Affine3d Tmotion = Told.inverse() * Tgt;
	Todo = Todo*Tmotion;
	
	if(Tmotion.translation().norm() > 1.0){ ///Not enough rotation for one meter distance - we reset
		Told = Tgt;
		cloud_old = cloud;
		return;
	}
	
	if(fabs(Tmotion.rotation().eulerAngles(0,1,2)[2])<(25.0*M_PI/180.0)) return;
	
	///At this point we should have two scans - not further that 1m appart from each other and the rotation difference is at least 20 degrees	
	
	
	float dy =offy;
	float dx = offx;
	float alpha = atan2(dy,dx);
	float L = sqrt(dx*dx+dy*dy);
	
	///Laser pose in base frame
	float lpx = L * cos(alpha);
	float lpy = L * sin(alpha);
	float lpa = offa;
	
	Eigen::Affine3d Ts = getAsAffine(lpx,lpy,lpa);
	
	ScanPair<pcl::PointXYZ> sp(cloud_old, Told, cloud,Tgt);
	pairs.push_back(sp);
	//sp.drawScans(Ts,win3D,gl_points );
	fprintf(stderr,"New cloud score=%lf\n", sp.scoreICP(Ts));
	Told = Tgt;
	cloud_old = cloud;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}

int main(int argc, char **argv){
	ros::init(argc, argv, "laser2d_extrinsic_calibration");
	
// #ifdef USE_VISUALIZATION_DEBUG	
// 	initializeScene();
// #endif
        ROS_ERROR_STREAM("------------ laser2d_extrinsic_calibration ---------------");
        ROS_ERROR_STREAM(" make sure to not use the --clock parameter while replaying a .bag file");
        ROS_ERROR_STREAM(" if you do the ros::spinOnce() call will simply wait... for ever");
        ROS_ERROR_STREAM("----------------------------------------------------------");


        ros::NodeHandle n;
	ros::NodeHandle nh;
	ros::NodeHandle paramHandle ("~");
	//TT.Tic();
	std::string input_laser_topic; 
	
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	/// Set the values from a config file
	//////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////
	paramHandle.param<std::string>("input_laser_topic", input_laser_topic, std::string("/base_scan"));
        paramHandle.param<std::string>("tf_odo_topic", tf_odo_topic, std::string("odom_base_link"));
        paramHandle.param<std::string>("tf_state_topic", tf_state_topic, std::string("state_base_link"));
        paramHandle.param<std::string>("tf_world_frame", tf_world_frame, std::string("world"));

	double sensor_pose_x, sensor_pose_y, sensor_pose_th;
	paramHandle.param<double>("sensor_pose_x", sensor_pose_x, 0.);
	paramHandle.param<double>("sensor_pose_y", sensor_pose_y, 0.);
	paramHandle.param<double>("sensor_pose_t", sensor_pose_th, 0.);
	int nb_pairs;
	double x_s, y_s, yaw_s, xy_i, yaw_i;
	paramHandle.param<int>("nb_pairs", nb_pairs, 70);
	paramHandle.param<double>("x_search_offset", x_s, 0.05);
	paramHandle.param<double>("y_search_offset", y_s, 0.05);
	paramHandle.param<double>("yaw_search_offset", yaw_s, 2*M_PI/180.0);
	paramHandle.param<double>("xy_incr", xy_i, 0.01);
	paramHandle.param<double>("yaw_incr", yaw_i, 0.05*M_PI/180.0);
	
        last_message = ros::Time::now();
	        
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	/// Set the subscribers and setup callbacks and message filters
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	ros::Subscriber scansub=nh.subscribe(input_laser_topic, 1, callback);

	offa = sensor_pose_th;
	offx = sensor_pose_x;
	offy = sensor_pose_y;
	
	has_sensor_offset_set = true;
	fprintf(stderr,"Sensor Pose = (%lf %lf %lf)\n",offx, offy, offa);	
	///Now lets create our publishers
	
	//ros::spin();

	ros::Rate r(20);
	unsigned int counter = 0;
        bool run = true;
	while(ros::ok()){
                ros::spinOnce();
		r.sleep();

		if(pairs.size()>nb_pairs) break;
		ros::Duration d = ros::Time::now() - last_message;
		if(d.toSec()>5.0 && pairs.size() > 0){
			fprintf(stderr,"No new messages for 5sec - break!\n");
			break;
		}
	}

	fprintf(stderr,"Starting optimization with initial estimate %lf %lf %lf\n", offx,offy,offa*180.0/M_PI);
	fprintf(stderr,"Number of scan pairs found : %i\n", (int)pairs.size());
	fprintf(stderr,"x,y,yaw offset : %lf,%lf,%lf\n", x_s, y_s, yaw_s);
	fprintf(stderr,"xy, yaw incr   : %lf,%lf\n", xy_i, yaw_i);

	//bruteForceOptimization(double ix, double iy, double iyaw, double x_s, double y_s, double yaw_s, double t_r, double yaw_r)
	bruteForceOptimization(offx, offy, offa, x_s, y_s, yaw_s, xy_i, yaw_i);
	
	fprintf(stderr,"END!\n");
	// while(win3D.isOpen()){
	// 	if (win3D.keyHit())
	// 	{
	// 		mrptKeyModifier kmods;
	// 		int key = win3D.getPushedKey(&kmods);
	// 		printf("Key pushed: %c (%i) - modifiers: 0x%04X\n",char(key),key,kmods);
	// 	}
	// 	usleep(100*1000);
	// }
		
}
