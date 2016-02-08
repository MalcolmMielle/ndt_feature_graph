#ifndef NDT_FEATURE_FUSER_HMT_HH
#define NDT_FEATURE_FUSER_HMT_HH

#include <ndt_visualisation/ndt_viz.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_map_hmt.h>
#include <ndt_registration/ndt_matcher_d2d_2d.h>
#include <ndt_registration/ndt_matcher_d2d.h>
#include <ndt_feature/ndt_matcher_d2d_fusion.h>
#include <eigen_conversions/eigen_msg.h>
#include <semrob_generic/motion_model2d.h>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>

//#define BASELINE

#include <ndt_feature/ndt_rviz.h>

#include <flirtlib_ros/flirtlib.h>
#include <flirtlib_ros/conversions.h>

#include <ndt_feature/flirtlib_utils.h>
#include <ndt_feature/ndt_feature_rviz.h>
#include <ndt_feature/conversions.h>
#include <ndt_feature/utils.h>

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

    NDTViz *viewer;
    FILE *fAddTimes, *fRegTimes;

    boost::shared_ptr<RansacFeatureSetMatcher> ransac_;

    InterestPointVec ptsPrev;
  int ctr;
  
  // Parameters
  
    bool checkConsistency;		  ///perform a check for consistency against initial estimate
    double max_translation_norm, max_rotation_norm;
    double sensor_range;
    bool be2D, doMultires, fuseIncomplete, beHMT;
    std::string prefix;
    std::string hmt_map_dir;

  pcl::PointCloud<pcl::PointXYZ> pointcloud_vis;
  
  class Params {
  public:
    Params() {
      // old params
      checkConsistency = false;		  ///perform a check for consistency against initial estimate
    
      // new params
      useNDT = true;
      useFeat = true;
      useOdom = true;
      neighbours = 0;
      stepcontrol = true;
      ITR_MAX = 30;
      DELTA_SCORE = 10e-4;
      globalTransf = true;
      loadCentroid = true;
      forceOdomAsEst = false;
      visualizeLocalCloud = false;
      fusion2d = false;
      allMatchesValid = false;
    }
    
    bool checkConsistency;
    
    bool useNDT;
    bool useFeat;
    bool useOdom;
    int neighbours;
    bool stepcontrol;
    int ITR_MAX;
    double DELTA_SCORE;
    bool globalTransf;
    bool loadCentroid;
    bool forceOdomAsEst;
    bool visualizeLocalCloud;
    bool fusion2d;
    bool allMatchesValid;
  
    friend std::ostream& operator<<(std::ostream &os, const NDTFeatureFuserHMT::Params &obj)
    {
      os << "\nuseNDT      : " << obj.useNDT;
      os << "\nuseFeat     : " << obj.useFeat;
      os << "\nuseOdom     : " << obj.useOdom;
      os << "\nneighbours  : " << obj.neighbours;
      os << "\nstepcontrol : " << obj.stepcontrol;
      os << "\nITR_MAX     : " << obj.ITR_MAX;
      os << "\nDELTA_SCORE : " << obj.DELTA_SCORE;
      os << "\nglobalTransf: " << obj.globalTransf;
      os << "\nloadCentroid: " << obj.loadCentroid;
      os << "\nforceOdomAsEst: " << obj.forceOdomAsEst;
      os << "\nvisualizeLocalCloud : " << obj.visualizeLocalCloud;
      os << "\nfusion2d    : " << obj.fusion2d;
      os << "\nallMatchesValid : " << obj.allMatchesValid;
      return os;
    }
  };
  
  Params params_;
  semrob_generic::MotionModel2d::Params motion_params_;

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
      localMapSize<<sensor_range_+3*resolution,sensor_range_+3*resolution,map_size_z_;
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

  void setParams(const Params &params) {
    params_ = params;
  }

  void setMotionParams(const semrob_generic::MotionModel2d::Params &params) {
    motion_params_ = params;
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
	map->initialize(Tnow.translation()(0),Tnow.translation()(1),0./*Tnow.translation()(2)*/,map_size_x,map_size_y,map_size_z);
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

      debug_markers_.clear();
      pcl::PointCloud<pcl::PointXYZ> cloud_orig(cloud);

        Eigen::Vector3d map_centroid;
        map->getCentroid(map_centroid[0], map_centroid[1], map_centroid[2]);
        std::cout << "map_centroid : " << map_centroid << std::endl;
      
      std::cout << "Tmotion : " << std::flush; lslgeneric::printTransf2d(Tmotion);
      // This is already given in the right frame. Add an odometry constraint by
      // utilizing two NDT distributions that are added.
      
      semrob_generic::MotionModel2d motion(motion_params_);
      semrob_generic::Pose2d relpose(Tmotion.translation()[0],
                                 Tmotion.translation()[1],
                                 Tmotion.rotation().eulerAngles(0,1,2)[2]);
      semrob_generic::Pose2dCov relposecov = motion.getPose2dCov(relpose);
      Eigen::Matrix3d odom_cov = relposecov.cov;
      odom_cov(2,2) = 0.01; // This is the height in the ndt feature vec and not rotational variance.
      lslgeneric::NDTCell* ndt_odom_cell = new lslgeneric::NDTCell();
      ndt_odom_cell->setMean(Eigen::Vector3d(0.,0.,0.));
      ndt_odom_cell->setCov(odom_cov);
      lslgeneric::NDTCell* ndt_odom_cell_prev = new lslgeneric::NDTCell();
      ndt_odom_cell_prev->setMean(Tmotion.translation());
      ndt_odom_cell_prev->setCov(odom_cov);
      std::cout << "covariance : " << odom_cov << std::endl;
      
      std::cout << "fuser params : " << params_ << std::endl;
      std::cout << "cloud.size() : " << cloud.size() << std::endl;

      Todom = Todom * Tmotion; //we track this only for display purposes!

#if 1
      Eigen::Affine3d Tinit;
      if (params_.globalTransf) {
        
        Tinit = Tnow;// * Tmotion;
      }
      else {
        //        Tinit = Tmotion;
        Tinit.setIdentity();
      }
      
      Eigen::Affine3d Tmotion_est;
      if (params_.globalTransf) {
        Tmotion_est = Tmotion;
      }
      else {
        Tmotion_est = Tnow*Tmotion;
      }
      Eigen::Affine3d global_rotation;
      Tinit.translation()(2) = 0.;
      Tmotion_est.translation()(2) = 0.;

      double t0=0,t1=0,t2=0,t3=0,t4=0,t5=0,t6=0;
      ///Set the cloud to sensor frame with respect to base
      //+++	    lslgeneric::transformPointCloudInPlace(sensor_pose, cloud);
      Eigen::Affine3d Tinit_sensor_pose = Tinit*sensor_pose;
      lslgeneric::transformPointCloudInPlace(Tinit_sensor_pose, cloud);
      
      t0 = getDoubleTime();
      ///Create global map
      lslgeneric::NDTMap ndglobal(new lslgeneric::LazyGrid(resolution));
      
      
      
      //+++ ndglobal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
      // DEBUG THIS FUNC...!
      
      std::cout << "Tnow : " <<  std::endl;
      lslgeneric::printTransf(Tnow);
      std::cout << "Tinit : " <<  std::endl;
      lslgeneric::printTransf(Tinit);
      std::cout << "Tinit_sensor_pose : " <<  std::endl;
      lslgeneric::printTransf(Tinit_sensor_pose);
      
      // Remove the z value...
      Tinit_sensor_pose.translation()[2] = 0.;
      std::cout << "Tinit_sensor_pose : " <<  std::endl;
      lslgeneric::printTransf(Tinit_sensor_pose);
      
      if (params_.loadCentroid) {
        if (params_.globalTransf) {
          ndglobal.loadPointCloudCentroid(cloud, Tinit_sensor_pose.translation(),
                                          map_centroid, localMapSize, sensor_range);
        }
        else {
          // This is more tricky...
          // 1) need to rotate the cloud to be aligned with a global frame... (Tmotion_est)
          // 2) need a translation (new map_centroid) that would align the current local map
          global_rotation = Tmotion_est;
          // Remove the rotation from Tmotion_est.
          Tmotion_est = Eigen::Translation3d(global_rotation.translation());
          global_rotation.translation() = Eigen::Vector3d(0.,0.,0.);
          lslgeneric::transformPointCloudInPlace(global_rotation, cloud);
          // The new centroid... select it to be as 'close' to 0,0,0 as possible.
          Eigen::Vector3d local_centroid = lslgeneric::computeLocalCentroid(map_centroid, Tmotion_est.translation(), resolution);
          std::cout << "local_centroid : " << local_centroid << std::endl;
          ndglobal.loadPointCloudCentroid(cloud, Tinit_sensor_pose.translation(),
                                          local_centroid, localMapSize, sensor_range);
        }
      }
      else {
        if (!params_.globalTransf) {
          ndglobal.guessSize(0,0,0,sensor_range,sensor_range,map_size_z);
        }
        ndglobal.loadPointCloud(cloud, sensor_range);
      }
      ndglobal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
      
      debug_markers_.push_back(ndt_visualisation::markerNDTCells(ndglobal, 2, "ndglobal"));
      {
        std::cout << "numberOfActiveCells : " << ndglobal.numberOfActiveCells() << std::endl;
        int size_x, size_y, size_z;
        ndglobal.getGridSize(size_x, size_y, size_z);
        std::cout << "x : " << size_x << "," << size_y << "," << size_z << std::endl;
        
        
      }
      t1 = getDoubleTime();
      
      // The feature handling goes here...
      Correspondences matches;
      OrientedPoint2D transform;
      std::vector<std::pair<int, int> > corr;
      Eigen::Matrix3d cov;
      cov << 0.0002, 0., 0., 0., 0.0002, 0., 0., 0., 0.0001; // Feature covarance...
      
      const double score_feature = ransac_->matchSets(ptsPrev, pts, transform, matches);
      Eigen::Affine3d Tfeat_sensor; // Local SENSOR coords...
      ndt_feature::convertOrientedPoint2DToEigen(transform, Tfeat_sensor);
      // Transform back to vechicle coords.
      Eigen::Affine3d Tfeat = sensor_pose * Tfeat_sensor * sensor_pose.inverse();
      
      std::cout<<"Tmotion : "<< Tmotion.translation().transpose()<<" "<<Tmotion.rotation().eulerAngles(0,1,2).transpose()<<std::endl;
      std::cout<<"Tfeat : "<< Tfeat.translation().transpose()<<" "<<Tfeat.rotation().eulerAngles(0,1,2).transpose()<<std::endl;
      
      bool consistent_features = true;
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
          //		checkConsistency = true;
          Eigen::Affine3d diff = (Tnow * Tmotion).inverse() * (Tnow * Tfeat);
          if((diff.translation().norm() > max_translation_norm/10. || 
              diff.rotation().eulerAngles(0,1,2).norm() > max_rotation_norm/4.) && checkConsistency){
            ROS_ERROR("****  NDTFuserHMT -- feature registration failure *****");
            
            consistent_features = false;
          } 
        }
        else {
          consistent_features = false;
        }
      }


      // NDT based feature matching comes here...
      lslgeneric::CellVector* cv_prev_sensor_frame = new lslgeneric::CellVector();
      lslgeneric::CellVector* cv_curr_sensor_frame = new lslgeneric::CellVector();
      
      bool use_odom_or_features = true;
      // Compute the NDT and the corr vec.
      if (params_.useFeat && consistent_features) {
        ndt_feature::convertCorrespondencesToCellvectorsFixedCovWithCorr(matches, cov, cv_curr_sensor_frame, cv_prev_sensor_frame, corr);
      }
      
      
      lslgeneric::NDTMap ndt_feat_prev_sensor_frame(cv_prev_sensor_frame);
      lslgeneric::NDTMap ndt_feat_curr_sensor_frame(cv_curr_sensor_frame);
      
      lslgeneric::NDTMap* ndt_feat_prev_vehicle_frame = ndt_feat_prev_sensor_frame.pseudoTransformNDTMap(sensor_pose);
      // The current frame is always in vehicle frame (to be moved Tinit in registration)-
      lslgeneric::NDTMap* ndt_feat_curr_vehicle_frame = ndt_feat_curr_sensor_frame.pseudoTransformNDTMap(sensor_pose);
      
      if (params_.useOdom) {
        // Add odometry (if enabled last)
        ROS_ERROR_STREAM("adding odometry...");
        addNDTCellToMap(ndt_feat_prev_vehicle_frame, ndt_odom_cell_prev);
        addNDTCellToMap(ndt_feat_curr_vehicle_frame, ndt_odom_cell);
        int tmp_size = corr.size(); 
        corr.push_back(std::pair<int,int>(tmp_size, tmp_size));
      }
      

      lslgeneric::NDTMap* ndt_feat_prev = ndt_feat_prev_vehicle_frame->pseudoTransformNDTMap(Tnow/**Tmotion*/);
      lslgeneric::NDTMap* ndt_feat_curr = ndt_feat_curr_vehicle_frame->pseudoTransformNDTMap(Tinit);
      
      //            ROS_ERROR_STREAM("corr.size() : " << corr.size());
      
      debug_markers_.push_back(ndt_visualisation::markerNDTCells(*ndt_feat_curr, 3, "feat_curr"));
      debug_markers_.push_back(ndt_visualisation::markerNDTCells(*ndt_feat_prev, 2, "feat_prev"));
      

      t2 = getDoubleTime();
      if (!params_.useFeat && !params_.useOdom) {
        // Both odom and feature is in the same pot here
        use_odom_or_features = false;
      }
      if (!params_.useOdom && !consistent_features) { // No odometry and no consistent features -> nothing to use...
        use_odom_or_features = false;
      }
      //		if(matcher.match( *map, ndlocal,Tinit,true) || fuseIncomplete){
      // Matching using Tmotion / Tfeat - local coords relative motion...

      bool match_ok = true;
      if (params_.fusion2d) {
        match_ok = lslgeneric::matchFusion2d(*map, ndglobal, *ndt_feat_prev, *ndt_feat_curr, corr, Tmotion_est, true, params_.useNDT, use_odom_or_features, params_.stepcontrol, params_.ITR_MAX, params_.neighbours, params_.DELTA_SCORE) || fuseIncomplete;
      }
      else {
        match_ok = lslgeneric::matchFusion(*map, ndglobal, *ndt_feat_prev, *ndt_feat_curr, corr, Tmotion_est, true, params_.useNDT, use_odom_or_features, params_.stepcontrol, params_.ITR_MAX, params_.neighbours, params_.DELTA_SCORE) || fuseIncomplete;
      }

      std::cout << "match_ok : " << match_ok << std::endl;
      if (params_.allMatchesValid) {
        match_ok = true;
      }

      if (match_ok) {

        // Recompute the covariance (based on the matching...)
        {
          lslgeneric::NDTMatcherD2D matcher_d2d;
            Eigen::MatrixXd matching_cov(6,6);
            matcher_d2d.covariance(*map, ndglobal, Tmotion_est, matching_cov);
            semrob_generic::Pose2dCov posecov;
            posecov.mean = semrob_generic::pose2dFromAffine3d(Tmotion_est);
            posecov.cov = semrob_generic::cov6toCov3(matching_cov);
            lslgeneric::printTransf2d(Tmotion_est);
            std::cout << "matching_cov : " << matching_cov << std::endl;
            semrob_generic::pose2dClearDependence(posecov);
            std::cout << "posecov : " << posecov << std::endl;
            debug_markers_.push_back(ndt_visualisation::markerMeanCovariance2d(posecov.mean, posecov.cov, 100., 1, -1));

            current_posecov.mean = semrob_generic::pose2dFromAffine3d(Tmotion_est);
            Eigen::Matrix3d prev_cov = current_posecov.cov;
            current_posecov.cov = prev_cov+posecov.cov; // Only works if the registration is done from local frame to global that is that Tmotion holds the complete motion and the covariance estimate is only the local one.
            std::cout << "current_posecov : " << current_posecov << std::endl;
            debug_markers_.push_back(ndt_visualisation::markerMeanCovariance2d(current_posecov.mean, current_posecov.cov, 1., 2, 0));

            std::cout << "relposecov : " << relposecov << std::endl;
            
        }


        t3 = getDoubleTime();

        if (!params_.globalTransf) { // Plot the nd maps in global frame...
          lslgeneric::NDTMap* ndglobal_matched = ndglobal.pseudoTransformNDTMap(Tmotion_est);
          debug_markers_.push_back(ndt_visualisation::markerNDTCells(*ndglobal_matched, 2, "ndglobal_matched"));
        }
        // Update the motion estimation with the orientation

        Eigen::Affine3d diff = (Tmotion_est).inverse() * Tmotion;
        if((diff.translation().norm() > max_translation_norm || 
            diff.rotation().eulerAngles(0,1,2).norm() > max_rotation_norm) && checkConsistency){
          fprintf(stderr,"****  NDTFuserHMT -- ALMOST DEFINATELY A REGISTRATION FAILURE *****\n");
          Tnow = Tnow * Tmotion;
        }else

        {
          if (params_.forceOdomAsEst) {
            ROS_ERROR("forcing odom...");
            Tnow = Tnow*Tmotion;
          }
          else {
            if (params_.globalTransf) {
              Tnow = Tnow*Tmotion_est;
            }
            else {
              if (params_.loadCentroid)
                Tnow = Tmotion_est * global_rotation; // global_rotation, used with centroids.
              else
                Tnow = Tmotion_est;
            }
          }
          //			lslgeneric::transformPointCloudInPlace(Tnow, cloud);
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
      
      std::cout << "Tmotion_est : ";
      lslgeneric::printTransf2d(Tmotion_est);
      Eigen::Affine3d spose = Tnow*sensor_pose;
      lslgeneric::transformPointCloudInPlace(spose, cloud_orig);
      //	    map->addPointCloudMeanUpdate(spose.translation(),cloud,localMapSize, 1e5, 1250, map_size_z/2, 0.06);
      
      map->addPointCloud(spose.translation(),cloud_orig, 0.06, 25); // Here, keep the raw cloud and add it here!
      map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, spose.translation(), 0.1);
      
      t5 = getDoubleTime();
      

      if (params_.visualizeLocalCloud)
        pointcloud_vis = cloud;
      else
        pointcloud_vis = cloud_orig;
      
      ptsPrev = pts;
      return Tnow;
      
      




	    //########################################################################
	    //#######################################################
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

  semrob_generic::Pose2dCov current_posecov;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      };
} // namespace

#endif
