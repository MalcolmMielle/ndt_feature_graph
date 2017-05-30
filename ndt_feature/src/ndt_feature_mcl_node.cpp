/**
* 3D NDT-MCL Node. 
* This application runs the ndt-mcl localization based on a map and laser scanner and odometry. 
* 
* The initialization is based now on ground truth pose information (should be replaced with manual input). 
* 
* The visualization depends on mrpt-gui 
* 
* More details about the algorithm:
* Jari Saarinen, Henrik Andreasson, Todor Stoyanov and Achim J. Lilienthal, Normal Distributions Transform Monte-Carlo Localization (NDT-MCL)
* IEEE/RSJ International Conference on Intelligent Robots and Systems November 3-8, 2013, Tokyo Big Sight, Japan
* 
* @author Jari Saarinen (jari.p.saarinen@gmail.com), mods by Todor Stoyanov
* @NOTE: experimental 3D version in progress 
* @TODO Global initialization possibility 
* Known issues: in launch file (or hard coded parameters) you have to set the same resolution for NDT map as is saved -- otherwise it wont work
*/

#include <ndt_visualisation/ndt_viz.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"
#include <pcl_conversions/pcl_conversions.h>

#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include "sensor_msgs/PointCloud2.h"

//#include "ndt_mcl/impl/ndt_mcl.hpp"
#include "ndt_mcl/3d_ndt_mcl.h"
#include <ndt_map/ndt_map.h>

#include <ndt_feature/ndt_feature_rviz.h>

#define SYNC_FRAMES 20

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> PointsOdomSync;
//TODO typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> PointsPoseSync;

class NDTMCL3DNode {

    private:
	ros::NodeHandle nh_;
	NDTMCL3D *ndtmcl;
        boost::mutex mcl_m,message_m;

        message_filters::Subscriber<sensor_msgs::PointCloud2> *points2_sub_;
        message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
        ros::Subscriber gt_sub;

	///Laser sensor offset
	Eigen::Affine3d sensorPoseT; //<<Sensor offset with respect to odometry frame
	Eigen::Affine3d Told,Todo,Todo_old,Tcum; //<<old and current odometry transformations
	Eigen::Affine3d initPoseT; //<<Sensor offset with respect to odometry frame

	bool hasSensorPose, hasInitialPose;
	bool isFirstLoad;
	bool forceSIR, do_visualize;
	bool saveMap;						///< indicates if we want to save the map in a regular intervals
	std::string mapName; ///<name and the path to the map
	std::string output_map_name;
	double resolution;
	double subsample_level;
	int pcounter;
	NDTViz ndt_viz;

	ros::Publisher mcl_pub; ///< The output of MCL is published with this!
  ros::Publisher marker_pub_;
  message_filters::Synchronizer< PointsOdomSync > *sync_po_;

  std::string tf_base_link, tf_sensor_link, points_topic, odometry_topic, odometry_frame;

  ros::Timer heartbeat_slow_visualization_;
  ros::Timer heartbeat_fast_visualization_;
bool do_pub_ndt_markers_;
  bool do_pub_particles_markers_;

  int numParticles;
  std::ofstream gt_file_;
  std::ofstream est_file_;
  std::ofstream est2d_file_;
  std::string gt_topic;

  bool use_initial_pose_from_gt;

    public:
	NDTMCL3DNode(ros::NodeHandle param_nh) {
	    

	    //////////////////////////////////////////////////////////
	    /// Prepare Pose offsets
	    //////////////////////////////////////////////////////////
	    bool use_sensor_pose, use_initial_pose;
	    double pose_init_x,pose_init_y,pose_init_z,
		   pose_init_r,pose_init_p,pose_init_t;
	    double sensor_pose_x,sensor_pose_y,sensor_pose_z,
		   sensor_pose_r,sensor_pose_p,sensor_pose_t;

	    param_nh.param<bool>("set_sensor_pose", use_sensor_pose, true);
	    param_nh.param<bool>("set_initial_pose", use_initial_pose, false);
            param_nh.param<bool>("set_initial_pose_from_gt", use_initial_pose_from_gt, false);

	    if(use_initial_pose) {
		///initial pose of the vehicle with respect to the map
		param_nh.param("pose_init_x",pose_init_x,0.);
		param_nh.param("pose_init_y",pose_init_y,0.);
		param_nh.param("pose_init_z",pose_init_z,0.);
		param_nh.param("pose_init_r",pose_init_r,0.);
		param_nh.param("pose_init_p",pose_init_p,0.);
		param_nh.param("pose_init_t",pose_init_t,0.);
		initPoseT =  Eigen::Translation<double,3>(pose_init_x,pose_init_y,pose_init_z)*
		    Eigen::AngleAxis<double>(pose_init_r,Eigen::Vector3d::UnitX()) *
		    Eigen::AngleAxis<double>(pose_init_p,Eigen::Vector3d::UnitY()) *
		    Eigen::AngleAxis<double>(pose_init_t,Eigen::Vector3d::UnitZ()) ;

		hasInitialPose=true;
	    } else {
		hasInitialPose=false;
	    }

	    if(use_sensor_pose) {
		///pose of the sensor with respect to the vehicle odometry frame
		param_nh.param("sensor_pose_x",sensor_pose_x,0.);
		param_nh.param("sensor_pose_y",sensor_pose_y,0.);
		param_nh.param("sensor_pose_z",sensor_pose_z,0.);
		param_nh.param("sensor_pose_r",sensor_pose_r,0.);
		param_nh.param("sensor_pose_p",sensor_pose_p,0.);
		param_nh.param("sensor_pose_t",sensor_pose_t,0.);
		hasSensorPose = true;
		sensorPoseT =  Eigen::Translation<double,3>(sensor_pose_x,sensor_pose_y,sensor_pose_z)*
		    Eigen::AngleAxis<double>(sensor_pose_r,Eigen::Vector3d::UnitX()) *
		    Eigen::AngleAxis<double>(sensor_pose_p,Eigen::Vector3d::UnitY()) *
		    Eigen::AngleAxis<double>(sensor_pose_t,Eigen::Vector3d::UnitZ()) ;
	    } else {
		hasSensorPose = false;
	    }

	    //////////////////////////////////////////////////////////
	    /// Prepare the map
	    //////////////////////////////////////////////////////////
	    param_nh.param<std::string>("map_file_name", mapName, std::string("basement.ndmap"));
	    param_nh.param<bool>("save_output_map", saveMap, true);
	    param_nh.param<std::string>("output_map_file_name", output_map_name, std::string("ndt_mapper_output.ndmap"));
	    param_nh.param<double>("map_resolution", resolution , 0.2);
	    param_nh.param<double>("subsample_level", subsample_level , 1);

            fprintf(stderr,"USING RESOLUTION %lf\n",resolution);
	    
	    lslgeneric::NDTMap ndmap(new lslgeneric::LazyGrid(resolution));
	    ndmap.loadFromJFF(mapName.c_str());

	    //////////////////////////////////////////////////////////
	    /// Prepare MCL object 
	    //////////////////////////////////////////////////////////

	    ndtmcl = new NDTMCL3D(resolution,ndmap,-5);
	    param_nh.param<bool>("forceSIR", forceSIR, false);
	    if(forceSIR) ndtmcl->forceSIR=true;

	    fprintf(stderr,"*** FORCE SIR = %d****",forceSIR);
	    param_nh.param<double>("SIR_varP_threshold", ndtmcl->SIR_varP_threshold, 0.006);
            param_nh.param<int>("SIR_max_iters_wo_resampling", ndtmcl->SIR_max_iters_wo_resampling, 25);
            param_nh.param<int>("numParticles", numParticles, 100);

            param_nh.getParam("motion_model", ndtmcl->motion_model);
            param_nh.getParam("motion_model_offset", ndtmcl->motion_model_offset);
            

            mcl_pub = nh_.advertise<nav_msgs::Odometry>("ndt_mcl",10);
	    
	    //////////////////////////////////////////////////////////
	    /// Prepare the callbacks and message filters
	    //////////////////////////////////////////////////////////
	    //the name of the TF link associated to the base frame / odometry frame    
	    param_nh.param<std::string>("tf_base_link", tf_base_link, std::string("/base_link"));
	    //the name of the tf link associated to the 3d laser scanner
	    param_nh.param<std::string>("tf_laser_link", tf_sensor_link, std::string("/velodyne_link"));
	    ///topic to wait for point clouds
	    param_nh.param<std::string>("points_topic",points_topic,"points");
	    ///topic to wait for odometry messages
	    param_nh.param<std::string>("odometry_topic",odometry_topic,"odometry");
            param_nh.param<std::string>("odometry_frame", odometry_frame, "/world");

	    points2_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_,points_topic,1);
	    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,odometry_topic,10);
	    sync_po_ = new message_filters::Synchronizer< PointsOdomSync >(PointsOdomSync(SYNC_FRAMES), *points2_sub_, *odom_sub_);
	    sync_po_->registerCallback(boost::bind(&NDTMCL3DNode::callback, this, _1, _2));

	    isFirstLoad=true;
	    pcounter =0;

            //////////////////////////////////////////////////////////
            /// Visualization
            //////////////////////////////////////////////////////////
            param_nh.param<bool>("do_visualize", do_visualize, true);
	    param_nh.param<bool>("do_pub_ndt_markers", do_pub_ndt_markers_, true);
            param_nh.param<bool>("do_pub_particles_markers", do_pub_particles_markers_, true);

            if (do_visualize) {
              ndt_viz.win3D->start_main_loop_own_thread();
            }
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 3);
            heartbeat_slow_visualization_   = nh_.createTimer(ros::Duration(1.0),&NDTMCL3DNode::publish_visualization_slow,this);
            heartbeat_fast_visualization_   = nh_.createTimer(ros::Duration(0.1),&NDTMCL3DNode::publish_visualization_fast,this);
    
            //////////////////////////////////////////////////////////
            /// Evaluation
            //////////////////////////////////////////////////////////
            std::string gt_filename, est_filename, est2d_filename;
            param_nh.param<std::string>("gt_topic",gt_topic,"");
            ROS_ERROR_STREAM("gt_topic : " << gt_topic);
            param_nh.param<std::string>("output_gt_file",  gt_filename, "loc_gt_pose.txt");
            param_nh.param<std::string>("output_est_file", est_filename, "loc_est_pose.txt"); 
            param_nh.param<std::string>("output_est2d_file", est2d_filename, "loc_est2d_pose.txt");
            if (gt_filename != std::string("")) {
              gt_file_.open(gt_filename.c_str());
            }
            if (est_filename != std::string("")) {
              est_file_.open(est_filename.c_str());
            }
            if (est2d_filename != std::string("")) {
              est2d_file_.open(est2d_filename.c_str());
            }
            
            if (!gt_file_.is_open() || !est_file_.is_open() || !est2d_file_.is_open())
            {
//               ROS_ERROR_STREAM("Failed to open : " << est_file_ << " | " << est2d_file_ << " | " << gt_file_); 
            }
                        
            if (gt_topic != std::string("")) 
            {
              ROS_ERROR_STREAM("Subscribing to : " << gt_topic);
              gt_sub = nh_.subscribe<nav_msgs::Odometry>(gt_topic,10,&NDTMCL3DNode::gt_callback, this);	
            }
        

	}
	~NDTMCL3DNode() {
          if (gt_file_.is_open())
            gt_file_.close();
          if (est_file_.is_open())
            est_file_.close();
          if (est2d_file_.is_open())
            est2d_file_.close();
          
	    delete points2_sub_;
	    delete odom_sub_;
	    delete sync_po_;
	    delete ndtmcl;
	}

    void publish_visualization_fast(const ros::TimerEvent &event) {
                if (do_pub_particles_markers_) 
                {
                  mcl_m.lock();
                  marker_pub_.publish(ndt_visualisation::markerParticlesNDTMCL3D(*ndtmcl, -1, "particles"));
                  mcl_m.unlock();
                }
    }

  void publish_visualization_slow(const ros::TimerEvent &event) {
                if (do_pub_ndt_markers_)
                {
                  // visualization_msgs::Marker markers_ndt;
                  // ndt_visualisation::markerNDTCells2(*(graph->getLastFeatureFuser()->map),
                  //                                    graph->getT(), 1, "nd_global_map_last", markers_ndt);
                  // marker_pub_.publish(markers_ndt);
                  mcl_m.lock();
                  marker_pub_.publish(ndt_visualisation::markerNDTCells(ndtmcl->map, 1, "nd_map"));
                  mcl_m.unlock();
                }
  }


	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//new callback: looks up transforms on TF
	void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in,
		const nav_msgs::Odometry::ConstPtr& odo_in)
	{
	    mcl_m.lock();
	    //compute new odo and point cloud
	    Eigen::Quaterniond qd;
	    Eigen::Affine3d Tm;
	    pcl::PointCloud<pcl::PointXYZ> cloud, cloudT, cloudTO;
	    
	    qd.x() = odo_in->pose.pose.orientation.x;
	    qd.y() = odo_in->pose.pose.orientation.y;
	    qd.z() = odo_in->pose.pose.orientation.z;
	    qd.w() = odo_in->pose.pose.orientation.w;

	    Todo = Eigen::Translation3d (odo_in->pose.pose.position.x,
		    odo_in->pose.pose.position.y,odo_in->pose.pose.position.z) * qd;
	    
	    pcl::fromROSMsg (*cloud_in, cloud);
	    
	    fprintf(stderr,"cloud has %d points\n",(int)cloud.points.size());		
	    //check if sensor pose is known, if not, look it up on TF
	    if(!hasSensorPose) {

	    }
	    //check if we have done iterations 
	    if(isFirstLoad) {
              //if not, check if initial robot pose has been set
		if(!hasInitialPose) {
		    //can't do anything, wait for pose message...
                  ROS_INFO("waiting for initial pose");
		    mcl_m.unlock();
		    return;
		}
		//initialize filter
		Eigen::Vector3d tr = initPoseT.translation();
		Eigen::Vector3d rot = initPoseT.rotation().eulerAngles(0,1,2);
		
		Todo_old=Todo;
		Tcum = initPoseT;

		ndtmcl->initializeFilter(tr[0], tr[1],tr[2],rot[0],rot[1],rot[2],0.5, 0.5, 0.1, 2.0*M_PI/180.0, 2.0*M_PI/180.0 ,2.0*M_PI/180.0, this->numParticles);
                //		ndtmcl->initializeFilter(tr[0], tr[1],tr[2],rot[0],rot[1],rot[2],2, 2, 0.3, 20.0*M_PI/180.0, 2.0*M_PI/180.0 ,2.0*M_PI/180.0, this->numParticles);
		//ndt_viz.plotNDTMap(&ndtmcl->map,0,1.0,1.0,true, false); 
                ndt_viz.plotNDTSAccordingToOccupancy(-1,&ndtmcl->map);
		isFirstLoad = false;
		mcl_m.unlock();
		return;
	    }
	    
	    Tm = Todo_old.inverse()*Todo;
	    if(Tm.translation().norm()<0.01 && fabs(Tm.rotation().eulerAngles(0,1,2)[2])<(0.5*M_PI/180.0)) {
		mcl_m.unlock();
		return;
	    }
	    
	    Tcum = Tcum*Tm;
	    Todo_old=Todo;

	    //cut off user defined slice form the sensor data
	    //transform point cloud from sensor pose into base pose
    
	    ///Set the cloud to sensor frame with respect to base
	    lslgeneric::transformPointCloudInPlace(sensorPoseT, cloud);
	    //cloudT = lslgeneric::transformPointCloud(Tcum, cloud);

	    //update filter -> + add parameter to subsample ndt map in filter step
	    ndtmcl->updateAndPredictEff(Tm, cloud, subsample_level);


	    //update visualization
	    ///DRAW STUFF///
	    if(do_visualize) {
              pcounter++;
		if(pcounter%500==0){
		    ndt_viz.clear();
		    ndt_viz.plotNDTSAccordingToOccupancy(-1,&ndtmcl->map);

		}

		//if(pcounter%10==0){

		ndt_viz.clearParticles();
		for(int i=0;i<ndtmcl->pf.size();i++){
		    double x,y,z;
		    ndtmcl->pf.pcloud[i].getXYZ(x,y,z);
		    ndt_viz.addParticle(x, y,z+0.5, 1.0, 1.0, 1.0);
		}

		Eigen::Affine3d mean = ndtmcl->pf.getMean();
		ndt_viz.addTrajectoryPoint(mean.translation()[0],mean.translation()[1],mean.translation()[2]+0.5,1.0,0,0);	
		Eigen::Vector3d tr = Tcum.translation();
		ndt_viz.addTrajectoryPoint(tr[0],tr[1],tr[2]+0.5,0.0,1.0,0);	

		//ndt_viz.addPointCloud(cloudT,1,0,0);
		ndt_viz.displayParticles();
		ndt_viz.displayTrajectory();
		ndt_viz.win3D->setCameraPointingToPoint(mean.translation()[0],mean.translation()[1],3.0);
		ndt_viz.repaint();
		//}
	    }
	    //publish pose
	    sendROSOdoMessage(ndtmcl->pf.getMean(),odo_in->header.stamp);
	    mcl_m.unlock();

            

	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//FIXME: this should be in 3D
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool sendROSOdoMessage(Eigen::Affine3d mean,ros::Time ts){
	    nav_msgs::Odometry O;
	    static int seq = 0;
	    O.header.stamp = ts;
	    O.header.seq = seq;
	    O.header.frame_id = odometry_frame;
	    O.child_frame_id = "/mcl_pose";

	    O.pose.pose.position.x = mean.translation()[0];
	    O.pose.pose.position.y = mean.translation()[1];
	    O.pose.pose.position.z = mean.translation()[2];
	    Eigen::Quaterniond q (mean.rotation());
	    tf::Quaternion qtf;
	    tf::quaternionEigenToTF (q, qtf);
	    O.pose.pose.orientation.x = q.x();
	    O.pose.pose.orientation.y = q.y();
	    O.pose.pose.orientation.z = q.z();
	    O.pose.pose.orientation.w = q.w();

	    seq++;
	    mcl_pub.publish(O);

	    static tf::TransformBroadcaster br;
	    tf::Transform transform;
	    transform.setOrigin( tf::Vector3(mean.translation()[0],mean.translation()[1], mean.translation()[2]) );

	    transform.setRotation( qtf );
	    br.sendTransform(tf::StampedTransform(transform, ts, "world", "mcl_pose"));

            if (est_file_.is_open()) {
              est_file_ << ts << " " << lslgeneric::transformToEvalString(mean);
            }
            if (est2d_file_.is_open()) {
              est2d_file_ << ts << " " << lslgeneric::transformToEval2dString(mean);
            }
	    return true;
	}

	// Callback
	void gt_callback(const nav_msgs::Odometry::ConstPtr& msg_in)
	{
	    Eigen::Quaterniond qd;
	    Eigen::Affine3d gt_pose;

	    qd.x() = msg_in->pose.pose.orientation.x;
	    qd.y() = msg_in->pose.pose.orientation.y;
	    qd.z() = msg_in->pose.pose.orientation.z;
	    qd.w() = msg_in->pose.pose.orientation.w;
	    
	    gt_pose = Eigen::Translation3d (msg_in->pose.pose.position.x,
		    msg_in->pose.pose.position.y,msg_in->pose.pose.position.z) * qd;
	     
            //	    ROS_INFO("got GT pose from GT track");
            if (gt_file_.is_open()) {
              gt_file_ << msg_in->header.stamp << " " << lslgeneric::transformToEvalString(gt_pose);
            }

	    // m.lock();
            if(use_initial_pose_from_gt && !hasInitialPose) {
              hasInitialPose = true;
              ROS_INFO("Set initial pose from GT track");
              initPoseT = gt_pose;
            }
	    // m.unlock();
	}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

};


int main(int argc, char **argv){
	ros::init(argc, argv, "NDT-MCL");
	ros::NodeHandle paramHandle ("~");
	NDTMCL3DNode mclnode(paramHandle);   	
	ros::spin();
	return 0;
}
