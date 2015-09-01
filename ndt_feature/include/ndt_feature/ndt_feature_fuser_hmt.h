#ifndef NDT_FEATURE_FUSER_HMT_HH
#define NDT_FEATURE_FUSER_HMT_HH

#include <ndt_visualisation/ndt_viz.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_map_hmt.h>
#include <ndt_registration/ndt_matcher_d2d_2d.h>
#include <ndt_registration/ndt_matcher_d2d.h>
#include <ndt_feature/ndt_matcher_d2d_fusion.h>
#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>

//#define BASELINE

#include <ndt_feature/ndt_rviz.h>

#include <flirtlib_ros/flirtlib.h>
#include <flirtlib_ros/conversions.h>

#include <ndt_feature/flirtlib_utils.h>
#include <ndt_feature/ndt_feature_rviz.h>
#include <ndt_feature/conversions.h>

namespace ndt_feature {
  /**
   * \brief This class fuses new point clouds into a common ndt map reference, keeping tack of the 
   * camera postion.
   * \author Jari, Todor
   */
class NDTFeatureFuserHMT{
  public:
    std::vector<visualization_msgs::Marker> debug_markers_;
    Eigen::Affine3d Tnow, Tlast_fuse, Todom; ///< current pose
    //lslgeneric::NDTMapHMT *map;  ///< da map
    lslgeneric::NDTMap *map;  ///< da map
    bool checkConsistency;		  ///perform a check for consistency against initial estimate
    double max_translation_norm, max_rotation_norm;
    double sensor_range;
    bool be2D, doMultires, fuseIncomplete, beHMT;
    int ctr;
    std::string prefix;
    std::string hmt_map_dir;
    NDTViz *viewer;
    FILE *fAddTimes, *fRegTimes;
	
    boost::shared_ptr<RansacFeatureSetMatcher> ransac_;

    InterestPointVec ptsPrev;

  NDTFeatureFuserHMT(double map_resolution, double map_size_x_, double map_size_y_, double map_size_z_, double sensor_range_ = 3, 
		     bool visualize_=false, bool be2D_=false, bool doMultires_=false, bool fuseIncomplete_=false, int max_itr=30, 
		     std::string prefix_="", bool beHMT_=true, std::string hmt_map_dir_="map", bool _step_control=true) : 
    ransac_(new RansacFeatureSetMatcher(0.0599, 0.9, 0.1, 0.6, 0.0499, false)) {
    ROS_INFO_STREAM("NDTFeatureFuserHMT - constructor");
      isInit = false;
      resolution = map_resolution;
      sensor_pose.setIdentity();
      checkConsistency = false;
      visualize = true;
      translation_fuse_delta = 0.05;
      rotation_fuse_delta = 0.01;
      max_translation_norm = 1.;
      max_rotation_norm = M_PI/4.;
      map_size_x = map_size_x_;
      map_size_y = map_size_y_;
      map_size_z = map_size_z_;
      visualize = visualize_;
      be2D = be2D_;
      sensor_range = sensor_range_;
      prefix = prefix_;
      doMultires = doMultires_;
      ctr =0;
      ROS_INFO_STREAM("using NDTViz");
        viewer = new NDTViz(visualize);
        if (visualize) {
          viewer->win3D->start_main_loop_own_thread(); // Very very ugly to start it here... FIX ME.
        }
      ROS_INFO_STREAM("using NDTViz - done");
      localMapSize<<sensor_range_,sensor_range_,map_size_z_;
      fuseIncomplete = fuseIncomplete_;
      matcher.ITR_MAX = max_itr;
      matcher2D.ITR_MAX = max_itr;
      matcher.step_control=_step_control;
      matcher2D.step_control=_step_control;
      beHMT = beHMT_;
      hmt_map_dir=hmt_map_dir_;

	    
      char fname[1000];
      snprintf(fname,999,"%s_addTime.txt",prefix.c_str());
      fAddTimes = fopen(fname,"w");

      std::cout<<"MAP: resolution: "<<resolution<<" size "<<map_size_x<<" "<<map_size_y<<" "<<map_size_z<<" sr "<<sensor_range<<std::endl;
    ROS_INFO_STREAM("NDTFeatureFuserHMT - constructor - done");

    }
    ~NDTFeatureFuserHMT()
      {
	delete map;
	delete viewer;
	if(fAddTimes!=NULL) fclose(fAddTimes);
	if(fRegTimes!=NULL) fclose(fRegTimes);
      }

    double getDoubleTime()
    {
      struct timeval time;
      gettimeofday(&time,NULL);
      return time.tv_sec + time.tv_usec * 1e-6;
    }
    void setSensorPose(Eigen::Affine3d spose){
      sensor_pose = spose;
    }
	
    bool wasInit()
    {
      return isInit;
    }

    bool saveMap() {
      if(!isInit) return false;
      if(map == NULL) return false;
      if(beHMT) {
	lslgeneric::NDTMapHMT *map_hmt = dynamic_cast<lslgeneric::NDTMapHMT*> (map);
	if(map_hmt==NULL) return false;
	return (map_hmt->writeTo()==0);
      } else {
	char fname[1000];
	snprintf(fname,999,"%s/%s_map.jff",hmt_map_dir.c_str(),prefix.c_str());
	return (map->writeToJFF(fname) == 0);
      }
    }

    /**
     * Set the initial position and set the first scan to the map
     */
  void initialize(Eigen::Affine3d initPos, pcl::PointCloud<pcl::PointXYZ> &cloud, const InterestPointVec& pts, bool preLoad=false){
      ///Set the cloud to sensor frame with respect to base
      lslgeneric::transformPointCloudInPlace(sensor_pose, cloud);
      lslgeneric::transformPointCloudInPlace(initPos, cloud);
      Tnow = initPos;

      ptsPrev = pts; // pts are always given in the sensor frame...

      //#ifdef BASELINE
      //#else
      if(beHMT) {
	map = new lslgeneric::NDTMapHMT(resolution,
						Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2),
						map_size_x,map_size_y,map_size_z,sensor_range,hmt_map_dir,true);
	if(preLoad) {
	  lslgeneric::NDTMapHMT *map_hmt = dynamic_cast<lslgeneric::NDTMapHMT*> (map);
	  std::cout<<"Trying to pre-load maps at "<<initPos.translation()<<std::endl;
	  map_hmt->tryLoadPosition(initPos.translation());
	}
      } else {
	map = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(resolution));
	map->initialize(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2),map_size_x,map_size_y,map_size_z);
      }
      //#endif
      map->addPointCloud(Tnow.translation(),cloud, 0.1, 100.0, 0.1);
      //map->addPointCloudMeanUpdate(Tnow.translation(),cloud,localMapSize, 1e5, 1250, map_size_z/2, 0.06);
      //map->addPointCloudMeanUpdate(Tnow.translation(),cloud,localMapSize, 0.1, 100.0, 0.1);
      map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, Tnow.translation(), 0.1);
      isInit = true;
      Tlast_fuse = Tnow;
      Todom = Tnow;
      if(visualize) 
	{
	  viewer->plotNDTSAccordingToOccupancy(-1,map);
	  viewer->plotLocalNDTMap(cloud,resolution);
	  viewer->addTrajectoryPoint(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2)+0.2,1,0,0);
	  viewer->addTrajectoryPoint(Todom.translation()(0),Todom.translation()(1),Todom.translation()(2)+0.2,0,1,0);
	  viewer->displayTrajectory();
	  viewer->setCameraPointing(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2)+3);
	  viewer->repaint();
	}
    }

    /**
     *
     *
     */
  Eigen::Affine3d update(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud, const InterestPointVec& pts){
      std::cerr << "in update()" << std::endl;
      if(!isInit){
	fprintf(stderr,"NDT-FuserHMT: Call Initialize first!!\n");
	return Tnow;
      }
#if 1

      // THE OLD FUSER - get this to run! ##########################################
      //#########################################################################


      	    Todom = Todom * Tmotion; //we track this only for display purposes!
	    double t0=0,t1=0,t2=0,t3=0,t4=0,t5=0,t6=0;
	    ///Set the cloud to sensor frame with respect to base
	    lslgeneric::transformPointCloudInPlace(sensor_pose, cloud);
	    t0 = getDoubleTime();
	    ///Create local map
	    lslgeneric::NDTMap ndlocal(new lslgeneric::LazyGrid(resolution));
	    ndlocal.guessSize(0,0,0,sensor_range,sensor_range,map_size_z);
	    ndlocal.loadPointCloud(cloud,sensor_range);
	    ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

	    t1 = getDoubleTime();
	    Eigen::Affine3d Tinit = Tnow * Tmotion;



	    // The feature handling goes here...
	    Correspondences matches;
	    OrientedPoint2D transform;
	    std::vector<std::pair<int, int> > corr;
	    Eigen::Matrix3d cov;
	    cov << 0.0002, 0., 0., 0., 0.0002, 0., 0., 0., 0.0001;
	
	    const double score_feature = ransac_->matchSets(ptsPrev, pts, transform, matches);
	    Eigen::Affine3d Tfeat_sensor; // Local SENSOR coords...
	    ndt_feature::convertOrientedPoint2DToEigen(transform, Tfeat_sensor);
	    // Transform back to vechicle coords.
	    Eigen::Affine3d Tfeat = sensor_pose * Tfeat_sensor * sensor_pose.inverse();
	    
	    std::cout<<"Tmotion : "<< Tmotion.translation().transpose()<<" "<<Tmotion.rotation().eulerAngles(0,1,2).transpose()<<std::endl;
	    std::cout<<"Tfeat : "<< Tfeat.translation().transpose()<<" "<<Tfeat.rotation().eulerAngles(0,1,2).transpose()<<std::endl;

	    bool use_features = true;
	    debug_markers_.clear();
	    {
	      geometry_msgs::Pose pose0;
	      tf::poseEigenToMsg(Tnow*sensor_pose, pose0); // PREV
	      debug_markers_.push_back(ndt_feature::interestPointMarkersFrameId(ptsPrev, pose0, 1, std::string("/world")));
	      
	      geometry_msgs::Pose pose1;
	      tf::poseEigenToMsg(Tinit*sensor_pose, pose1); // CURRENT
	      debug_markers_.push_back(ndt_feature::interestPointMarkersFrameId(pts, pose1, 0, std::string("/world")));
	      
	      if (matches.size() > 0) {
		// Seems that the order is off.
		debug_markers_.push_back(ndt_feature::correspondenceMarkers (matches, 
									     pose1,
									     pose0,
									     std::string("/world")));
		// Check that this makes sence (not to much difference compared to the odometry...
		checkConsistency = true;
		Eigen::Affine3d diff = (Tnow * Tmotion).inverse() * (Tnow * Tfeat);
		if((diff.translation().norm() > max_translation_norm/10. || 
		    diff.rotation().eulerAngles(0,1,2).norm() > max_rotation_norm/4.) && checkConsistency){
		  ROS_ERROR("****  NDTFuserHMT -- feature registration failure *****");
		  
		  use_features = false;
		} 
	      }
	      else {
		use_features = false;
	      }
	    }

	    // NDT based matching comes here...
	    lslgeneric::CellVector* cv_prev = new lslgeneric::CellVector();
	    lslgeneric::CellVector* cv_curr = new lslgeneric::CellVector();
	
	    // Compute the NDT and the corr vec.
	    ndt_feature::convertCorrespondencesToCellvectorsFixedCovWithCorr(matches, cov, cv_curr, cv_prev, corr);
		
	    lslgeneric::NDTMap ndt_feat_prev_sensor_frame(cv_prev);
	    lslgeneric::NDTMap ndt_feat_curr_sensor_frame(cv_curr);
	    
	    lslgeneric::NDTMap* ndt_feat_prev = ndt_feat_prev_sensor_frame.pseudoTransformNDTMap(Tnow*sensor_pose);
	    lslgeneric::NDTMap* ndt_feat_curr = ndt_feat_curr_sensor_frame.pseudoTransformNDTMap(sensor_pose);
		    
	    debug_markers_.push_back(ndt_visualisation::markerNDTCells(*ndt_feat_curr, 3));
	    debug_markers_.push_back(ndt_visualisation::markerNDTCells(*ndt_feat_prev, 2));


	    t2 = getDoubleTime();
	    bool useNDT = false;
	    bool useFeat = use_features;
	    bool step_control = true;
	    
	    //		if(matcher.match( *map, ndlocal,Tinit,true) || fuseIncomplete){
	    if (lslgeneric::matchFusion(*map, ndlocal, *ndt_feat_prev, *ndt_feat_curr, corr, Tinit, true, useNDT, useFeat, step_control) || fuseIncomplete) {
		    t3 = getDoubleTime();
		    lslgeneric::NDTMap* ndlocal_matched = ndlocal.pseudoTransformNDTMap(Tinit);
		    debug_markers_.push_back(ndt_visualisation::markerNDTCells(*ndlocal_matched, 2));

		    Eigen::Affine3d diff = (Tnow * Tmotion).inverse() * Tinit;
		    if((diff.translation().norm() > max_translation_norm || 
				diff.rotation().eulerAngles(0,1,2).norm() > max_rotation_norm) && checkConsistency && !useFeat){
			fprintf(stderr,"****  NDTFuserHMT -- ALMOST DEFINATELY A REGISTRATION FAILURE *****\n");
			Tnow = Tnow * Tmotion;
		    }else{
			Tnow = Tinit;
			lslgeneric::transformPointCloudInPlace(Tnow, cloud);
			Eigen::Affine3d diff_fuse = Tlast_fuse.inverse()*Tnow;
			if(diff_fuse.translation().norm() > translation_fuse_delta ||
				diff_fuse.rotation().eulerAngles(0,1,2).norm() > rotation_fuse_delta)
			{
			    //std::cout<<"F: "<<spose.translation().transpose()<<" "<<spose.rotation().eulerAngles(0,1,2).transpose()<<std::endl;
			    t4 = getDoubleTime();
			    //map->addPointCloudMeanUpdate(spose.translation(),cloud,localMapSize, 1e5, 25, 2*map_size_z, 0.06);
			    //map->addPointCloud(spose.translation(),cloud, 0.06, 25);
			    //map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, spose.translation(), 0.1);
			    //t4 = getDoubleTime();
			    //std::cout<<"match: "<<t3-t2<<" addPointCloud: "<<t5-t4<<" ndlocal "<<t1-t0<<" total: "<<t5-t0<<std::endl;

			    debug_markers_.push_back(ndt_visualisation::markerNDTCells(*map, 1));


			    Tlast_fuse = Tnow;
			    if(visualize) //&&ctr%20==0) 
			    {
				if(ctr%20==0) {
				    viewer->plotNDTSAccordingToOccupancy(-1,map);
				    viewer->plotLocalNDTMap(cloud,resolution);
				}
				viewer->addTrajectoryPoint(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2),1,0,0);
				viewer->addTrajectoryPoint(Todom.translation()(0),Todom.translation()(1),Todom.translation()(2),0,1,0);
				viewer->displayTrajectory();
				viewer->setCameraPointing(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2)+3);
				viewer->repaint();
			    }
			    ctr++;
			}
		    }
		}else{
		    t3 = getDoubleTime();
		    Tnow = Tnow * Tmotion;
		}
		
	    Eigen::Affine3d spose = Tnow*sensor_pose;
	    //	    map->addPointCloudMeanUpdate(spose.translation(),cloud,localMapSize, 1e5, 1250, map_size_z/2, 0.06);
	    map->addPointCloud(spose.translation(),cloud, 0.06, 25);
	    map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, spose.translation(), 0.1);

	    t5 = getDoubleTime();
	    

		ptsPrev = pts;
		return Tnow;






	    //########################################################################
	    //#######################################################
#endif 
#if 0
      // ---------------------------------------------- START -------------------------------
      {
	// Do the matching
	Eigen::Affine3d Tinit;

	Tinit = Tnow * Tmotion; // Global coords
	Correspondences matches;
	OrientedPoint2D transform;
	std::vector<std::pair<int, int> > corr;
	Eigen::Matrix3d cov;
	cov << 0.001, 0., 0., 0., 0.001, 0., 0., 0., 0.001;
	std::cout << "pts.size() : " << pts.size() << std::endl;
	std::cout << "ptsPrev.size() : " << pts.size() << std::endl;
	
	const double score_feature = ransac_->matchSets(ptsPrev, pts, transform, matches);
	std::cout << "matches.size() : " << matches.size() << std::endl;
	// Convert the transform to Eigen... -> need also to derermine if this was successful or not.
	std::cout << "score_feature : " << score_feature << std::endl;
		
	Eigen::Affine3d Tfeat_sensor; // Local SENSOR coords...
	ndt_feature::convertOrientedPoint2DToEigen(transform, Tfeat_sensor);

	// Transform back to vechicle coords.
	Eigen::Affine3d Tfeat = sensor_pose * Tfeat_sensor * sensor_pose.inverse();

	std::cout<<"Tmotion : "<< Tmotion.translation().transpose()<<" "<<Tmotion.rotation().eulerAngles(0,1,2).transpose()<<std::endl;
 	std::cout<<"Tfeat : "<< Tfeat.translation().transpose()<<" "<<Tfeat.rotation().eulerAngles(0,1,2).transpose()<<std::endl;


	bool use_features = true;
	debug_markers_.clear();
	{
	  geometry_msgs::Pose pose0;
	  tf::poseEigenToMsg(Tnow*sensor_pose, pose0); // PREV
	  debug_markers_.push_back(ndt_feature::interestPointMarkersFrameId(ptsPrev, pose0, 1, std::string("/world")));

	  geometry_msgs::Pose pose1;
	  tf::poseEigenToMsg(Tinit*sensor_pose, pose1); // CURRENT
	  debug_markers_.push_back(ndt_feature::interestPointMarkersFrameId(pts, pose1, 0, std::string("/world")));

	  debug_markers_.push_back(ndt_feature::correspondenceMarkers (matches, 
								       pose1,
								       pose0,
								       std::string("/world")));
	  
	  if (matches.size() > 0) {
	    // Seems that the order is off.
	    // Check that this makes sence (not to much difference compared to the odometry...
	    checkConsistency = true;
	    Eigen::Affine3d diff = (Tnow * Tmotion).inverse() * (Tnow * Tfeat);
	    if((diff.translation().norm() > max_translation_norm || 
		diff.rotation().eulerAngles(0,1,2).norm() > max_rotation_norm) && checkConsistency){
	      ROS_ERROR("****  NDTFuserHMT -- feature registration failure *****");
	      
	      use_features = false;
	    } 
	  }
	  else {
	    use_features = false;
	  }
	}
	
	/* { */
	/*   geometry_msgs::Pose pose0; */
	/*   pose0.position.x = 0.; */
	/*   pose0.position.y = 0.; */
	/*   pose0.position.z = 0.; */
	/*   pose0.orientation = tf::createQuaternionMsgFromYaw(0.); */
	/*   debug_markers_.push_back(ndt_feature::interestPointMarkersFrameId(ptsPrev, pose0, 0, std::string("/base_laser_link"))); */
	/*   // This will use tf (the /fuser topic) to get the transform, despite of trying to get the correct time stamps this don't play along. */
	/* } */
	

	// NDT based matching comes here...
	lslgeneric::CellVector* cv_prev = new lslgeneric::CellVector();
	lslgeneric::CellVector* cv_curr = new lslgeneric::CellVector();
	

		// Compute the NDT and the corr vec.
	ndt_feature::convertCorrespondencesToCellvectorsFixedCovWithCorr(matches, cov, cv_prev, cv_curr, corr);
		

	lslgeneric::NDTMap ndt_feat_prev_sensor_frame(cv_prev);
	lslgeneric::NDTMap ndt_feat_curr_sensor_frame(cv_curr);
	
	lslgeneric::NDTMap* ndt_feat_prev = ndt_feat_prev_sensor_frame.pseudoTransformNDTMap(sensor_pose);
	lslgeneric::NDTMap* ndt_feat_curr = ndt_feat_curr_sensor_frame.pseudoTransformNDTMap(sensor_pose);
	

	lslgeneric::transformPointCloudInPlace(sensor_pose, cloud);
	
	///Create local map
	lslgeneric::NDTMap ndlocal(new lslgeneric::LazyGrid(resolution));
	ndlocal.guessSize(0,0,0,sensor_range,sensor_range,map_size_z);
	ndlocal.loadPointCloud(cloud,sensor_range);
	ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

		
	bool useNDT = true;
	bool useFeat = use_features;
	bool step_control = true;


	if (useFeat) {
	  debug_markers_.push_back(ndt_visualisation::markerNDTCells(ndt_feat_curr_sensor_frame, 3));
	  debug_markers_.push_back(ndt_visualisation::markerNDTCells(ndt_feat_prev_sensor_frame, 2));

	  debug_markers_.push_back(ndt_visualisation::markerNDTCells(*ndt_feat_curr, 3));
	  debug_markers_.push_back(ndt_visualisation::markerNDTCells(*ndt_feat_prev, 2));
	}

	// Great, attempt to do the ndt matching...
	// Start by verifying the feature based version...
	//	  lslgeneric::NDTMatcherFeatureD2D matcher(corr);
	//	  matcher.match(*ndt_feat_curr, *ndt_feat_prev, Tmotion);
	/* std::cout<<"Tmotion : "<< Tmotion.translation().transpose()<<" "<<Tmotion.rotation().eulerAngles(0,1,2).transpose()<<std::endl; */

	Eigen::Affine3d Tfusion = Tinit;

	bool use_odom = false;
	lslgeneric::NDTMap* ndlocal_prematched = ndlocal.pseudoTransformNDTMap(Tinit);

	//	if (lslgeneric::matchFusion<pcl::PointXYZ, pcl::PointXYZ>(*map, ndlocal, *ndt_feat_curr, *ndt_feat_prev, corr, Tfusion, true, useNDT, useFeat, step_control) || fuseIncomplete) {
	//	if(matcher2D.match( *map, ndlocal,Tfusion,true) || fuseIncomplete){
	if(matcher.match( *map, ndlocal,Tfusion,true) || fuseIncomplete){
	  
	  // Check the consistency...
	  Eigen::Affine3d diff = (Tnow * Tmotion).inverse() * (Tfusion);
	  if((diff.translation().norm() > max_translation_norm || 
	      diff.rotation().eulerAngles(0,1,2).norm() > max_rotation_norm) && checkConsistency){
	    ROS_ERROR("****  NDTFuserHMT -- NDT fusion registration error... *****");
	    use_odom = true;
	  } 
	}

	std::cout<<"Tfusion : "<< Tfusion.translation().transpose()<<" "<<Tfusion.rotation().eulerAngles(0,1,2).transpose()<<std::endl;


	if (!use_odom) {
	  Tnow = Tfusion;
	}
	else {
	  Tnow = Tnow*Tmotion;
	}

	std::cout<<"Tnow final : "<< Tnow.translation().transpose()<<" "<<Tnow.rotation().eulerAngles(0,1,2).transpose()<<std::endl;


	Tlast_fuse = Tnow;

	lslgeneric::transformPointCloudInPlace(Tmotion, cloud);
	Eigen::Affine3d spose = Tnow*sensor_pose;

	//map->addPointCloud(Tnow.translation(),cloud, 0.1, 100.0, 0.1);
	//map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, Tnow.translation(), 0.1);

	map->addPointCloudMeanUpdate(spose.translation(),cloud,localMapSize, 1e5, 1250, map_size_z/2, 0.06);
	

	debug_markers_.push_back(ndt_visualisation::markerNDTCells(*map, 1));

 	lslgeneric::NDTMap* ndlocal_matched = ndlocal.pseudoTransformNDTMap(Tfusion);
	debug_markers_.push_back(ndt_visualisation::markerNDTCells(*ndlocal_matched, 2));
	debug_markers_.push_back(ndt_visualisation::markerNDTCells(*ndlocal_prematched, 3));


	if(visualize) //&&ctr%20==0) 
	  {
	    if(ctr%20==0) {
	      viewer->plotNDTSAccordingToOccupancy(-1,map); 
	      viewer->plotLocalNDTMap(cloud,resolution); 
	    }
	    viewer->addTrajectoryPoint(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2),1,0,0);
	    viewer->addTrajectoryPoint(Todom.translation()(0),Todom.translation()(1),Todom.translation()(2),0,1,0);
	    viewer->displayTrajectory();
	    viewer->setCameraPointing(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2)+3);
	    viewer->repaint();	
	  }
	ctr++;
	

	ptsPrev = pts;
	delete cv_prev;
	delete cv_curr;			    
      }
      // ------------------------------------------- END -------------------------------------
      return Tnow;



      Todom = Todom * Tmotion; //we track this only for display purposes!
      double t0=0,t1=0,t2=0,t3=0,t4=0,t5=0,t6=0;
      ///Set the cloud to sensor frame with respect to base
      lslgeneric::transformPointCloudInPlace(sensor_pose, cloud);
      t0 = getDoubleTime();
      ///Create local map
      lslgeneric::NDTMap ndlocal(new lslgeneric::LazyGrid(resolution));
      ndlocal.guessSize(0,0,0,sensor_range,sensor_range,map_size_z);
      ndlocal.loadPointCloud(cloud,sensor_range);
      ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
      //pass through ndlocal and set all cells with vertically pointing normals to non-gaussian :-O
      /*SpatialIndex *index = ndlocal.getMyIndex();
	typename SpatialIndex::CellVectorItr it = index->begin();
	while (it != index->end())
	{
	NDTCell *cell = dynamic_cast<NDTCell*> (*it);
	if(cell!=NULL)
	{
	if(cell->hasGaussian_)
	{
	if(cell->getClass() == NDTCell::HORIZONTAL) {
	cell->hasGaussian_ = false;
	}
	}
	}
	it++;
	}*/

      t1 = getDoubleTime();
      Eigen::Affine3d Tinit = Tnow * Tmotion;
      if(doMultires) {
	//create two ndt maps with resolution = 3*resolution (or 5?)
	lslgeneric::NDTMap ndlocalLow(new lslgeneric::LazyGrid(3*resolution));
	ndlocalLow.guessSize(0,0,0,sensor_range,sensor_range,map_size_z);
	ndlocalLow.loadPointCloud(cloud,sensor_range);
	ndlocalLow.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

	lslgeneric::NDTMap mapLow(new lslgeneric::LazyGrid(3*resolution));
	//add distros
	double cx,cy,cz;
	if(!map->getCentroid(cx, cy, cz)){
	  fprintf(stderr,"Centroid NOT Given-abort!\n");
	}
	mapLow.initialize(cx,cy,cz,3*map_size_x,3*map_size_y,map_size_z);

	std::vector<lslgeneric::NDTCell*> ndts;
	ndts = map->getAllCells(); //this copies cells?
	
	for(int i=0; i<ndts.size(); i++)	
	  {
	    lslgeneric::NDTCell *cell = ndts[i];
	    if(cell!=NULL)
	      {
		if(cell->hasGaussian_)
		  {
		    Eigen::Vector3d m = cell->getMean();	
		    Eigen::Matrix3d cov = cell->getCov();
		    unsigned int nump = cell->getN();
		    mapLow.addDistributionToCell(cov, m,nump);
		  }
	      }
	    delete cell;
	  }
	//do match
	if(matcher2D.match( mapLow, ndlocalLow,Tinit,true)){
	  //if success, set Tmotion to result
	  t2 = getDoubleTime();
	  //std::cout<<"success: new initial guess! t= "<<t2-t1<<std::endl;
	} else {
	  Tinit = Tnow * Tmotion;
	}	    
      }
	    
      if(be2D) {
	t2 = getDoubleTime();
		
	// Do the matching
	Correspondences matches;
	OrientedPoint2D transform;
	std::vector<std::pair<int, int> > corr;
	Eigen::Matrix3d cov;
	cov << 0.02, 0., 0., 0., 0.02, 0., 0., 0., 0.01;
	std::cout << "pts.size() : " << pts.size() << std::endl;
	std::cout << "ptsPrev.size() : " << pts.size() << std::endl;
	
	const double score_feature = ransac_->matchSets(ptsPrev, pts, transform, matches);
	std::cout << "matches.size() : " << matches.size() << std::endl;
	lslgeneric::CellVector* cv_prev = new lslgeneric::CellVector();
	lslgeneric::CellVector* cv_curr = new lslgeneric::CellVector();
		
	// Compute the NDT and the corr vec.
	ndt_feature::convertCorrespondencesToCellvectorsFixedCovWithCorr(matches, cov, cv_prev, cv_curr, corr);
		

	lslgeneric::NDTMap ndt_feat_prev_sensor_frame(cv_prev);
	lslgeneric::NDTMap ndt_feat_curr_sensor_frame(cv_curr);
	
	// ndt_feat_prev -> should be in global coordinates, e.g. moved to the previous vehicle pose = Tnow and moved to the sensor frame.
	// ndt_feat_curr -> should be in base coordinates of the vehicle, e.g. moved to the sensor frame.
	lslgeneric::NDTMap* ndt_feat_prev = ndt_feat_prev_sensor_frame.pseudoTransformNDTMap(Tnow*sensor_pose);
	lslgeneric::NDTMap* ndt_feat_curr = ndt_feat_prev_sensor_frame.pseudoTransformNDTMap(sensor_pose);
		
	bool useNDT = false;
	bool useFeat = true;
	bool step_control = true;

	debug_markers_.clear();
	{
	  geometry_msgs::Pose pose0;
	  tf::poseEigenToMsg(Tnow*sensor_pose, pose0);
	  debug_markers_.push_back(ndt_feature::interestPointMarkersFrameId(ptsPrev, pose0, 1, std::string("/world")));   // Do this instead of...
	}
	{
	  geometry_msgs::Pose pose0;
	  pose0.position.x = 0.;
	  pose0.position.y = 0.;
	  pose0.position.z = 0.;
	  pose0.orientation = tf::createQuaternionMsgFromYaw(0.);
	  //	  debug_markers_.push_back(ndt_feature::interestPointMarkersFrameId(ptsPrev, pose0, 0, std::string("/base_laser_link")));
	  // This will use tf (the /fuser topic) to get the transform, despite of trying to get the correct time stamps this don't play along.
	}
	{
	}
	

	debug_markers_.push_back(ndt_visualisation::markerNDTCells(ndlocal, 2));
	debug_markers_.push_back(ndt_visualisation::markerNDTCells(*ndt_feat_curr, 3));
	debug_markers_.push_back(ndt_visualisation::markerNDTCells(*ndt_feat_prev, 2));
	Eigen::Affine3d Tinit2 = Tinit;
	if (lslgeneric::matchFusion<pcl::PointXYZ, pcl::PointXYZ>(*map, ndlocal, *ndt_feat_prev, *ndt_feat_curr, corr, Tinit, true, useNDT, useFeat, step_control) || fuseIncomplete) {
		  
	  //		if(matcher2D.match( *map, ndlocal,Tinit,true) || fuseIncomplete){
	  Tinit = Tinit2; // REMOVE ME! -> This looks a bit funny - check why... do a clean incremental registration class instead (which this currently is anyway - there is some problems in the transformations(!!!).
	  if (matches.size() > 0)
	  {
	    ROS_ERROR("debuging...");
	    geometry_msgs::Pose pose0; // OLD POSE
	    tf::poseEigenToMsg(Tnow*sensor_pose, pose0);
	    
	    geometry_msgs::Pose pose1; // NEW POSE
	    tf::poseEigenToMsg(Tinit*sensor_pose, pose1);
	    debug_markers_.push_back(ndt_feature::correspondenceMarkers (matches, 
									 pose0,
									 pose1,
									 std::string("/world")));
	  }


	  t3 = getDoubleTime();
	  Eigen::Affine3d diff = (Tnow * Tmotion).inverse() * Tinit;
	  if((diff.translation().norm() > max_translation_norm || 
	      diff.rotation().eulerAngles(0,1,2).norm() > max_rotation_norm) && checkConsistency){
	    fprintf(stderr,"****  NDTFuserHMT -- ALMOST DEFINATELY A REGISTRATION FAILURE *****\n");
	    ROS_ERROR("****  NDTFuserHMT -- ALMOST DEFINATELY A REGISTRATION FAILURE *****");
	    Tnow = Tnow * Tmotion;
	  }else{
	    Tnow = Tinit;

	    debug_markers_.push_back(ndt_visualisation::markerNDTCells(*(ndt_feat_prev_sensor_frame.pseudoTransformNDTMap(Tnow*sensor_pose)), 3));
	    

	    lslgeneric::transformPointCloudInPlace(Tnow, cloud);
	    Eigen::Affine3d spose = Tnow*sensor_pose;
	    Eigen::Affine3d diff_fuse = Tlast_fuse.inverse()*Tnow;
	    if(diff_fuse.translation().norm() > translation_fuse_delta ||
	       diff_fuse.rotation().eulerAngles(0,1,2).norm() > rotation_fuse_delta)
	      {
		//std::cout<<"F: "<<spose.translation().transpose()<<" "<<spose.rotation().eulerAngles(0,1,2).transpose()<<std::endl;
		t4 = getDoubleTime();
		map->addPointCloudMeanUpdate(spose.translation(),cloud,localMapSize, 1e5, 1250, map_size_z/2, 0.06);
		t5 = getDoubleTime();
		//map->addPointCloudMeanUpdate(spose.translation(),cloud,localMapSize, 1e5, 25, 2*map_size_z, 0.06);
		//map->addPointCloud(spose.translation(),cloud, 0.06, 25);
		//map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, spose.translation(), 0.1);
		//t4 = getDoubleTime();
		//std::cout<<"match: "<<t3-t2<<" addPointCloud: "<<t5-t4<<" ndlocal "<<t1-t0<<" total: "<<t5-t0<<std::endl;
		Tlast_fuse = Tnow;
		if(visualize) //&&ctr%20==0) 
		  {
		    if(ctr%20==0) {
		      viewer->plotNDTSAccordingToOccupancy(-1,map); 
		      viewer->plotLocalNDTMap(cloud,resolution); 
		    }
		    viewer->addTrajectoryPoint(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2),1,0,0);
		    viewer->addTrajectoryPoint(Todom.translation()(0),Todom.translation()(1),Todom.translation()(2),0,1,0);
		    viewer->displayTrajectory();
		    viewer->setCameraPointing(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2)+3);
		    viewer->repaint();	
		  }
		ctr++;
	      }
	  }
	  t3 = getDoubleTime();
	  //	  Tnow = Tnow * Tmotion;
		  
	}else{
	}
	ptsPrev = pts;
	delete cv_prev;
	delete cv_curr;			    
      }
      else
	{
		
	  t2 = getDoubleTime();
	  if(matcher.match( *map, ndlocal,Tinit,true) || fuseIncomplete){
	    t3 = getDoubleTime();
	    Eigen::Affine3d diff = (Tnow * Tmotion).inverse() * Tinit;
		    
	    if((diff.translation().norm() > max_translation_norm || 
		diff.rotation().eulerAngles(0,1,2).norm() > max_rotation_norm) && checkConsistency){
	      fprintf(stderr,"****  NDTFuserHMT -- ALMOST DEFINATELY A REGISTRATION FAILURE *****\n");
	      Tnow = Tnow * Tmotion;
	      //save offending map:
	      //map->writeToJFF("map.jff");
	      //ndlocal.writeToJFF("local.jff");
	    }else{
	      Tnow = Tinit;
	      //Tnow = Tnow * Tmotion;
	      lslgeneric::transformPointCloudInPlace(Tnow, cloud);
	      Eigen::Affine3d spose = Tnow*sensor_pose;
	      Eigen::Affine3d diff_fuse = Tlast_fuse.inverse()*Tnow;
	      if(diff_fuse.translation().norm() > translation_fuse_delta ||
		 diff_fuse.rotation().eulerAngles(0,1,2).norm() > rotation_fuse_delta)
		{
		  //std::cout<<"F: "<<spose.translation().transpose()<<" "<<spose.rotation().eulerAngles(0,1,2).transpose()<<std::endl;
		  t4 = getDoubleTime();
		  map->addPointCloudMeanUpdate(spose.translation(),cloud,localMapSize, 1e5, 1250, map_size_z/2, 0.06);
		  t5 = getDoubleTime();
		  //map->addPointCloudMeanUpdate(spose.translation(),cloud,localMapSize, 1e5, 25, 2*map_size_z, 0.06);
		  //map->addPointCloud(spose.translation(),cloud, 0.06, 25);
		  //map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, spose.translation(), 0.1);
		  //t4 = getDoubleTime();
		  //std::cout<<"match: "<<t3-t2<<" addPointCloud: "<<t5-t4<<" ndlocal "<<t1-t0<<" total: "<<t5-t0<<std::endl;
		  Tlast_fuse = Tnow;
		  if(visualize) //&&ctr%20==0) 
		    {
		      viewer->plotNDTSAccordingToOccupancy(-1,map); 
		      viewer->plotLocalNDTMap(cloud,resolution); 
		    }
		  ctr++;
		}
	    }
	  }else{
	    t3 = getDoubleTime();
	    Tnow = Tnow * Tmotion;
	  }
	}
	    
      t6 = getDoubleTime();
      if(fAddTimes!=NULL) {
	fprintf(fAddTimes,"%lf %lf %lf\n",t3-t2,t5-t4,t6-t0);
	fflush(fAddTimes);
      }

      return Tnow;
#endif
    }

  private:
    bool isInit;

    double resolution; ///< resolution of the map
    double map_size;
	
    double translation_fuse_delta, rotation_fuse_delta;
    double map_size_x;
    double map_size_y;
    double map_size_z;
    bool visualize;

    Eigen::Affine3d sensor_pose;
    lslgeneric::NDTMatcherD2D matcher;
    lslgeneric::NDTMatcherD2D_2D matcher2D;
    Eigen::Vector3d localMapSize;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      };
} // namespace

#endif
