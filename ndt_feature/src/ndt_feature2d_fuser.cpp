// Test code to evaluate how laser2d features such as the FLIRT library could paly along with NDT.
#include <ndt_feature/ndt_feature_fuser_hmt.h>
//#include <ndt_fuser/ndt_fuser_hmt.h>
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"
#include <cstdio>
#include <Eigen/Eigen>
#include <fstream>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/circular_buffer.hpp>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_srvs/Empty.h>

#include <flirtlib_ros/flirtlib.h>
#include <flirtlib_ros/conversions.h>

#include <ndt_feature/flirtlib_utils.h>
#include <ndt_feature/conversions.h>
#include <ndt_feature/ndt_feature_rviz.h>
#include <ndt_feature/ndt_feature_fuser_hmt.h>

#include <ndt_feature/ndt_feature_frame.h>
#include <ndt_feature/ndt_offline_mapper.h>

#ifndef SYNC_FRAMES
#define SYNC_FRAMES 20
#endif
/** \brief A ROS node which implements an NDTFuser or NDTFuserHMT object
  * \author Todor Stoyanov
  * 
  */
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> LaserOdomSync;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, geometry_msgs::PoseStamped> LaserPoseSync;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> PointsOdomSync;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> PointsPoseSync;

class NDTFeatureFuserNode {

    protected:
	// Our NodeHandle
	ros::NodeHandle nh_;

	message_filters::Subscriber<sensor_msgs::PointCloud2> *points2_sub_;
	message_filters::Subscriber<sensor_msgs::LaserScan> *laser_sub_;
	message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
	ros::Subscriber gt_sub;

	// Components for publishing
	tf::TransformBroadcaster tf_;
	ros::Publisher output_pub_;
        ros::Publisher pointcloud_pub_;
  ros::Publisher pointcloud2_pub_;
	Eigen::Affine3d pose_, T, sensor_pose_;

        tf::TransformListener listener;

	unsigned int nb_added_clouds_;
	double varz;
	
	boost::mutex m, message_m;
	ndt_feature::NDTFeatureFuserHMT *fuser;
	std::string points_topic, laser_topic, map_dir, map_name, odometry_topic, 
		    world_frame, fuser_frame, init_pose_frame, gt_topic;
	double size_x, size_y, size_z, resolution, sensor_range, min_laser_range_;
	bool visualize, match2D, matchLaser, beHMT, useOdometry, plotGTTrack, 
	     initPoseFromGT, initPoseFromTF, initPoseSet;

    bool offline;
    int offline_nb_readings;
    std::vector<ndt_feature::NDTFeatureFrame> offline_frames;
    int offline_ctr;
    std::vector<std::pair<size_t, size_t> > offline_matches;

	double pose_init_x,pose_init_y,pose_init_z,
	       pose_init_r,pose_init_p,pose_init_t;
	double sensor_pose_x,sensor_pose_y,sensor_pose_z,
	       sensor_pose_r,sensor_pose_p,sensor_pose_t;
	laser_geometry::LaserProjection projector_;

	message_filters::Synchronizer< LaserOdomSync > *sync_lo_;
	message_filters::Synchronizer< LaserPoseSync > *sync_lp_;
	
	message_filters::Synchronizer< PointsOdomSync > *sync_po_;
	message_filters::Synchronizer< PointsPoseSync > *sync_pp_;
	ros::ServiceServer save_map_;
        ros::Publisher marker_pub_;

	Eigen::Affine3d last_odom, this_odom;

    // Flirtlib
    boost::shared_ptr<SimpleMinMaxPeakFinder> peak_finder_;
    boost::shared_ptr<HistogramDistance<double> > histogram_dist_;
    boost::shared_ptr<Detector> detector_;
    boost::shared_ptr<DescriptorGenerator> descriptor_;

    public:
	// Constructor
    NDTFeatureFuserNode(ros::NodeHandle param_nh) : nb_added_clouds_(0),
        peak_finder_(ndt_feature::createPeakFinder()),
        histogram_dist_(new SymmetricChi2Distance<double>()),
        detector_(ndt_feature::createDetector(peak_finder_.get())),
        descriptor_(ndt_feature::createDescriptor(histogram_dist_.get()))
	{
	    ///topic to wait for point clouds, if available
	    param_nh.param<std::string>("points_topic",points_topic,"points");
	    ///topic to wait for laser scan messages, if available
	    param_nh.param<std::string>("laser_topic",laser_topic,"laser_scan");

	    ///if using the HMT fuser, NDT maps are saved in this directory. 
	    ///a word of warning: if you run multiple times with the same directory, 
	    ///the old maps are loaded automatically
	    param_nh.param<std::string>("map_directory",map_dir,"map");
	    param_nh.param<std::string>("map_name_prefix",map_name,"");
	    
	    ///initial pose of the vehicle with respect to the map
	    param_nh.param("pose_init_x",pose_init_x,0.);
	    param_nh.param("pose_init_y",pose_init_y,0.);
	    param_nh.param("pose_init_z",pose_init_z,0.);
	    param_nh.param("pose_init_r",pose_init_r,0.);
	    param_nh.param("pose_init_p",pose_init_p,0.);
	    param_nh.param("pose_init_t",pose_init_t,0.);
	    
	    ///pose of the sensor with respect to the vehicle odometry frame
	    param_nh.param("sensor_pose_x",sensor_pose_x,0.);
	    param_nh.param("sensor_pose_y",sensor_pose_y,0.);
	    param_nh.param("sensor_pose_z",sensor_pose_z,0.);
	    param_nh.param("sensor_pose_r",sensor_pose_r,0.);
	    param_nh.param("sensor_pose_p",sensor_pose_p,0.);
	    param_nh.param("sensor_pose_t",sensor_pose_t,0.);
	    
	    ///size of the map in x/y/z. if using HMT, this is the size of the central tile
	    param_nh.param("size_x_meters",size_x,10.);
	    param_nh.param("size_y_meters",size_y,10.);
	    param_nh.param("size_z_meters",size_z,10.);
	    
	    ///range to cutoff sensor measurements
	    param_nh.param("sensor_range",sensor_range,3.);
	    ///range to cutoff sensor measurements
	    param_nh.param("min_laser_range",min_laser_range_,0.1);
	    
	    //map resolution
	    param_nh.param("resolution",resolution,0.10);
	    param_nh.param("laser_variance_z",varz,resolution/4);

	    ///visualize in a local window
	    param_nh.param("visualize",visualize,true);
	    ///only mathc with 3dof
	    param_nh.param("match2D",match2D,false);
	    ///use HMT grid or simple grid.
	    param_nh.param("beHMT",beHMT,false);
	    ///use standard odometry messages for initial guess 
	    param_nh.param("useOdometry",useOdometry,false);
	    ///topic to wait for laser scan messages, if available
	    param_nh.param<std::string>("odometry_topic",odometry_topic,"odometry");
	    ///if we want to compare to a ground truth topic
	    param_nh.param("plotGTTrack",plotGTTrack,false);
	    param_nh.param<std::string>("gt_topic",gt_topic,"groundtruth");
	    ///if we want to get the initial pose of the vehicle relative to a different frame
	    param_nh.param("initPoseFromGT",initPoseFromGT,false);
	    //get it from TF?
	    param_nh.param("initPoseFromTF",initPoseFromTF,false);
	    //the frame to initialize to
	    param_nh.param<std::string>("init_pose_frame",init_pose_frame,"/state_base_link");
	    //the world frame
	    param_nh.param<std::string>("world_frame",world_frame,"/world");
	    //our frame
	    param_nh.param<std::string>("fuser_frame",fuser_frame,"/fuser");
 
	    ///enable for LaserScan message input
	    param_nh.param("matchLaser",matchLaser,false);
            
            param_nh.param<int>("offline_nb_readings", offline_nb_readings, 0);

            ndt_feature::NDTFeatureFuserHMT::Params fuser_params;
            param_nh.param<bool>("fuser_useNDT", fuser_params.useNDT, true);
            param_nh.param<bool>("fuser_useFeat", fuser_params.useFeat, true);
            param_nh.param<bool>("fuser_useOdom", fuser_params.useOdom, true);
            param_nh.param<int>("fuser_neighbours", fuser_params.neighbours, 2);
            param_nh.param<bool>("fuser_stepcontrol", fuser_params.stepcontrol, true);
            param_nh.param<int>("fuser_ITR_MAX", fuser_params.ITR_MAX, 30);
            param_nh.param<double>("fuser_DELTA_SCORE", fuser_params.DELTA_SCORE, 0.0001);
            param_nh.param<bool>("fuser_globalTransf", fuser_params.globalTransf, true);
            param_nh.param<bool>("fuser_loadCentroid", fuser_params.loadCentroid, true);
            param_nh.param<bool>("fuser_forceOdomAsEst", fuser_params.forceOdomAsEst, false);
            param_nh.param<bool>("fuser_visualizeLocalCloud", fuser_params.visualizeLocalCloud, false);
            param_nh.param<bool>("fuser_fusion2d", fuser_params.fusion2d, false);

            offline = false;
            if (offline_nb_readings > 0)
                offline = true;
            offline_ctr = 0;


            pose_ =  Eigen::Translation<double,3>(pose_init_x,pose_init_y,pose_init_z)*
		Eigen::AngleAxis<double>(pose_init_r,Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxis<double>(pose_init_p,Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxis<double>(pose_init_t,Eigen::Vector3d::UnitZ()) ;
	    
	    sensor_pose_ =  Eigen::Translation<double,3>(sensor_pose_x,sensor_pose_y,sensor_pose_z)*
		Eigen::AngleAxis<double>(sensor_pose_r,Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxis<double>(sensor_pose_p,Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxis<double>(sensor_pose_t,Eigen::Vector3d::UnitZ()) ;

	    if(matchLaser) match2D=true;
	    fuser = new ndt_feature::NDTFeatureFuserHMT(resolution,size_x,size_y,size_z,
		    sensor_range, visualize,match2D, false, false, 30, map_name, beHMT, map_dir, true);

	    fuser->setSensorPose(sensor_pose_);
            fuser->setParams(fuser_params);
            
            semrob_generic::MotionModel2d::Params motion_params;
            motion_params.Cd = 0.001;
            motion_params.Ct = 0.001;
            motion_params.Dd = 0.005;
            motion_params.Dt = 0.005;
            motion_params.Td = 0.001;
            motion_params.Tt = 0.001;
            fuser->setMotionParams(motion_params);

	    if(!matchLaser) {
		points2_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_,points_topic,1);
		if(useOdometry) {
		    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,odometry_topic,10);
		    sync_po_ = new message_filters::Synchronizer< PointsOdomSync >(PointsOdomSync(SYNC_FRAMES), *points2_sub_, *odom_sub_);
		    sync_po_->registerCallback(boost::bind(&NDTFeatureFuserNode::points2OdomCallback, this, _1, _2));
		} else {
		    points2_sub_->registerCallback(boost::bind( &NDTFeatureFuserNode::points2Callback, this, _1));
		}
	    } else {
		laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_,laser_topic,2);
		if(useOdometry) {
		    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,odometry_topic,10);
		    sync_lo_ = new message_filters::Synchronizer< LaserOdomSync >(LaserOdomSync(SYNC_FRAMES), *laser_sub_, *odom_sub_);
		    sync_lo_->registerCallback(boost::bind(&NDTFeatureFuserNode::laserOdomCallback, this, _1, _2));

		} else {
		    laser_sub_->registerCallback(boost::bind( &NDTFeatureFuserNode::laserCallback, this, _1));
		}
	    }
	    save_map_ = param_nh.advertiseService("save_map", &NDTFeatureFuserNode::save_map_callback, this);

	    if(plotGTTrack) {
		gt_sub = nh_.subscribe<nav_msgs::Odometry>(gt_topic,10,&NDTFeatureFuserNode::gt_callback, this);	
	    }
	    initPoseSet = false;
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
            pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("offline_map", 1000);
            pointcloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 1000);
        }

	~NDTFeatureFuserNode()
	{
	    delete fuser;
	}

    
    void processFeatureFrame(pcl::PointCloud<pcl::PointXYZ> &cloud, const InterestPointVec& pts, Eigen::Affine3d Tmotion, const ros::Time &frameTime) {
        m.lock();
	    if (nb_added_clouds_  == 0)
	    {
		ROS_INFO("initializing fuser map. Init pose from GT? %d, TF? %d", initPoseFromGT, initPoseFromTF);
		if(initPoseFromGT || initPoseFromTF) {
		    //check if initial pose was set already 
		    if(!initPoseSet) {
			ROS_WARN("skipping frame, init pose not acquired yet!");
			m.unlock();
			return;
		    }
		}
		ROS_INFO("Init pose is (%lf,%lf,%lf)", pose_.translation()(0), pose_.translation()(1), 
			pose_.rotation().eulerAngles(0,1,2)(0));
		fuser->initialize(pose_,cloud,pts);
		nb_added_clouds_++;
	    } else {
		nb_added_clouds_++;
		pose_ = fuser->update(Tmotion,cloud,pts);
	    }
	    m.unlock();

	    tf::Transform transform;
#if ROS_VERSION_MINIMUM(1,9,0)
//groovy
	    tf::transformEigenToTF(pose_, transform);
#else
//fuerte
	    tf::TransformEigenToTF(pose_, transform);
#endif
	    tf_.sendTransform(tf::StampedTransform(transform, frameTime, world_frame, fuser_frame));

            // Add some drawing
            if (fuser->wasInit()) {

                for (size_t i = 0; i < fuser->debug_markers_.size(); i++) {
                    fuser->debug_markers_[i].header.stamp = frameTime;
                    marker_pub_.publish(fuser->debug_markers_[i]);
                }

//                marker_pub_.publish(ndt_visualisation::markerNDTCells(*fuser->map, 1));
            }
    }


	
	bool save_map_callback(std_srvs::Empty::Request  &req,
		std_srvs::Empty::Response &res ) {

	    bool ret = false;
	    ROS_INFO("Saving current map to map directory %s", map_dir.c_str());
	    m.lock();
	    ret = fuser->saveMap(); 
	    m.unlock();

	    return ret;
	}


	// Callback
	void points2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg_in)
	{
            assert(false);

	};
	
	// Callback
	void points2OdomCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_in,
		  const nav_msgs::Odometry::ConstPtr& odo_in)
	{
            assert(false);
        };
	
	// Callback
	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
	{
          //          ROS_ERROR("laserCallback()");
	    sensor_msgs::PointCloud2 cloud;
	    pcl::PointCloud<pcl::PointXYZ> pcl_cloud, pcl_cloud_unfiltered;
	    Eigen::Affine3d Tm;
            Eigen::Affine3d Tgt;
            ros::Time frame_time = msg_in->header.stamp;


            // Use the TF to get the pose here...
            // Could be problematic with the approximate time?
            
            // Ask TF for the pose of the laser scan.
            

            // The odometry that should be provided is in vehicle coordinates - the /base_link
            {
                tf::StampedTransform transform;
                try {   
                    listener.waitForTransform("/world", "/odom_base_link",
                                              frame_time, ros::Duration(3.0));
                    listener.lookupTransform("/world", "/odom_base_link",
                                             frame_time, transform);
                    
                }
                catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                    return;
                }
                tf::transformTFToEigen( transform, this_odom);
            }

            // Ground truth
            {
                tf::StampedTransform transform;
                try {   
                    listener.waitForTransform("/world", "/state_base_link",
                                              frame_time, ros::Duration(3.0));
                    listener.lookupTransform("/world", "/state_base_link",
                                             frame_time, transform);
                    
                }
                catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                    return;
                }
                tf::transformTFToEigen( transform, Tgt );
            }


            // Need to get the incremental update.
            if (nb_added_clouds_  == 0)
	    {
		Tm.setIdentity();
	    } else {
                Tm = last_odom.inverse()*this_odom;

                if (Tm.translation().norm() < 0.02 && Tm.rotation().eulerAngles(0,1,2).norm() < 0.2)
                    return;
            }
            


            last_odom = this_odom;

            
            projector_.projectLaser(*msg_in, cloud);
	    message_m.unlock();
	    
	    pcl::fromROSMsg (cloud, pcl_cloud_unfiltered);

	    pcl::PointXYZ pt;
	    //add some variance on z
	    for(int i=0; i<pcl_cloud_unfiltered.points.size(); i++) {
		pt = pcl_cloud_unfiltered.points[i];
		if(sqrt(pt.x*pt.x+pt.y*pt.y) > min_laser_range_) {
		    pt.z += varz*((double)rand())/(double)INT_MAX;
		    pcl_cloud.points.push_back(pt);
		}
	    }
          // Compute the flirt features
            boost::shared_ptr<LaserReading> reading = flirtlib_ros::fromRos(*msg_in);
            InterestPointVec pts;
            detector_->detect(*reading, pts);
            // Descriptor computation
            BOOST_FOREACH (InterestPoint* p, pts) 
                p->setDescriptor(descriptor_->describe(*p, *reading));
            

            // Get the time when the laser was taken, this is what should be published...
            if (offline) {
                if (offline_ctr < offline_nb_readings) {
                    ndt_feature::NDTFeatureFrame f;
                    f.pc = pcl_cloud;
                    f.pts = pts;
                    f.odom = this_odom;
                    f.gt = Tgt;
                    offline_frames.push_back(f);
                    ROS_INFO_STREAM(" number of frames : " << offline_frames.size());
                    ndt_feature::publishMarkerNDTFeatureFrames(offline_frames, marker_pub_);

                    offline_ctr++;
                    if (offline_ctr >= offline_nb_readings) {
                        // Run the matcher(!)
                        ROS_INFO("RUNNING OFFLINE MATCHER");
                        ndt_feature::mapBuilderISAMOffline(offline_frames, sensor_pose_, offline_matches);
                    }
                }
                else {
                    ROS_INFO(".");
//                    ndt_feature::publishMarkerNDTFeatureFrames(offline_frames, marker_pub_);
                    marker_pub_.publish(ndt_feature::posePointsMarkerNDTFeatureFrames(offline_frames, 1, 1, std::string("pose_est"), false));
//                    marker_pub_.publish(ndt_feature::posePointsMarkerNDTFeatureFrames(offline_frames, 1, 1, std::string("pose_odom"), true));

                    pcl::PointCloud<pcl::PointXYZ> cloud;
                    ndt_feature::getEstimatedCloudNDTFeatureFrames(offline_frames, sensor_pose_, cloud);
                    sensor_msgs::PointCloud2 cloud_msg; 
                    pcl::toROSMsg(cloud, cloud_msg );
                    cloud_msg.header.frame_id = std::string("/world");
                    pointcloud_pub_.publish(cloud_msg );
                    marker_pub_.publish(ndt_feature::featureMatchesMarkerNDTFeatureFrames(offline_frames, offline_matches, 0, 1, std::string("matches"), false));
//                    marker_pub_.publish(ndt_feature::featureMatchesMarkerNDTFeatureFrames(offline_frames, offline_matches, 1, 2, std::string("matches_odom"), true));

                }
            }
            else {
                this->processFeatureFrame(pcl_cloud,pts, Tm, frame_time);
            }
            {
              sensor_msgs::PointCloud2 cloud_msg; 
              pcl::toROSMsg(fuser->pointcloud_vis, cloud_msg );
              cloud_msg.header.frame_id = std::string("/world");
              pointcloud2_pub_.publish(cloud_msg );
            }
	};
	
	// Callback
	void laserOdomCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in,
		  const nav_msgs::Odometry::ConstPtr& odo_in)
	{

          //          ROS_ERROR("laserOdomCallback()");
  

	    Eigen::Quaterniond qd;
	    sensor_msgs::PointCloud2 cloud;
	    pcl::PointCloud<pcl::PointXYZ> pcl_cloud, pcl_cloud_unfiltered;
	    Eigen::Affine3d Tm;

	    message_m.lock();
	    qd.x() = odo_in->pose.pose.orientation.x;
	    qd.y() = odo_in->pose.pose.orientation.y;
	    qd.z() = odo_in->pose.pose.orientation.z;
	    qd.w() = odo_in->pose.pose.orientation.w;
	    
	    this_odom = Eigen::Translation3d (odo_in->pose.pose.position.x,
		    odo_in->pose.pose.position.y,odo_in->pose.pose.position.z) * qd;

	    //std::cout<<"AT: "<<this_odom.translation().transpose()<<" "<<this_odom.rotation().eulerAngles(0,1,2)[2] << std::endl;
	    
	    if (nb_added_clouds_  == 0)
	    {
		Tm.setIdentity();
	    } else {
		Tm = last_odom.inverse()*this_odom;
		//std::cout<<"delta from last update: "<<Tm.translation().transpose()<<" "<<Tm.rotation().eulerAngles(0,1,2)[2] << std::endl;
		//if(Tm.translation().norm()<0.2 && fabs(Tm.rotation().eulerAngles(0,1,2)[2])<(5*M_PI/180.0)) {
		//    message_m.unlock();
		//    return;
		//}
	    }
	    last_odom = this_odom;

	    projector_.projectLaser(*msg_in, cloud);
	    message_m.unlock();
	    
	    pcl::fromROSMsg (cloud, pcl_cloud_unfiltered);

	    pcl::PointXYZ pt;
	    //add some variance on z
	    for(int i=0; i<pcl_cloud_unfiltered.points.size(); i++) {
		pt = pcl_cloud_unfiltered.points[i];
		if(sqrt(pt.x*pt.x+pt.y*pt.y) > min_laser_range_) {
		    pt.z += varz*((double)rand())/(double)INT_MAX;
		    pcl_cloud.points.push_back(pt);
		}
	    }
	    //ROS_INFO("Got laser and odometry!");
          // Compute the flirt features
            boost::shared_ptr<LaserReading> reading = flirtlib_ros::fromRos(*msg_in);
            InterestPointVec pts;
            detector_->detect(*reading, pts);
            // Descriptor computation
            BOOST_FOREACH (InterestPoint* p, pts) 
                p->setDescriptor(descriptor_->describe(*p, *reading));
            

            // Get the time when the laser was taken, this is what should be published...
            ros::Time frame_time = msg_in->header.stamp;
            // ros::Time frame_time = odo_in->header.stamp; // TODO, check how this really works with the sync stuff.
            this->processFeatureFrame(pcl_cloud,pts, Tm, frame_time);
                
	};
	
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
	     
	    //ROS_INFO("got GT pose from GT track");
	    m.lock();
	    if(initPoseFromGT && !initPoseSet) {
		initPoseSet = true;
		pose_ = gt_pose;
		ROS_INFO("Set initial pose from GT track");
	    }
            if (visualize) {
              fuser->viewer->addTrajectoryPoint(gt_pose.translation()(0),gt_pose.translation()(1),gt_pose.translation()(2)+0.2,1,1,1);
              fuser->viewer->displayTrajectory();
              fuser->viewer->repaint();	
            }
	    m.unlock();
	}

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ndt_feature2d_fuser");

    ros::NodeHandle param("~");
    NDTFeatureFuserNode t(param);
    ros::spin();

    return 0;
}

