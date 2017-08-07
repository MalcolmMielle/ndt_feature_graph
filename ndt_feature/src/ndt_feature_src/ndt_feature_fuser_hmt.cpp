
#include "ndt_feature/ndt_feature_fuser_hmt.h"

bool ndt_feature::NDTFeatureFuserHMT::saveMap()
{
	if(!isInit) return false;
	if(map == NULL) return false;
	if(params_.beHMT) {
	lslgeneric::NDTMapHMT *map_hmt = dynamic_cast<lslgeneric::NDTMapHMT*> (map);
	if(map_hmt==NULL) return false;
		return (map_hmt->writeTo()==0);
	} else {
		char fname[1000];
		snprintf(fname,999,"%s/%s_map.jff",params_.hmt_map_dir.c_str(),params_.prefix.c_str());
		return (map->writeToJFF(fname) == 0);
	}
}
  

bool ndt_feature::NDTFeatureFuserHMT::save(const std::string &fileName) {
    bool ret;
    std::string nd_file = fileName + ".jff";
    std::string feat_file = fileName + ".feat";
    ret = (map->writeToJFF(nd_file.c_str()) == 0);
    ret &= (featuremap.save(feat_file));
    return ret;
}

bool ndt_feature::NDTFeatureFuserHMT::load(const std::string &fileName) {
    bool ret = true;
    
    std::string nd_file = fileName + ".jff";
    std::string feat_file = fileName + ".feat";
    // Do the initializations etc.
    Eigen::Affine3d T;
    T.setIdentity();
    map = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(params_.resolution));
    std::cout << "loading : " << nd_file << std::endl;
    if (map->loadFromJFF(nd_file.c_str()) != 0) {
      std::cerr << "failed to load : " << nd_file << std::endl;
      ret = false;
    }
    std::cerr << "loading feature map : " << feat_file << std::endl;
    if (featuremap.load(feat_file) == false) {
      std::cerr << "failed to load : " << feat_file << std::endl;
      ret = false;
    }
    return ret;
}


  // const std::vector<visualization_msgs::Marker>& getVisualizationMarkers() const {
  //   return debug_markers_;
  // }

pcl::PointCloud<pcl::PointXYZ>& ndt_feature::NDTFeatureFuserHMT::getVisualizationCloud() {
    return pointcloud_vis;
}

  

    /**
     * Set the initial position and set the first scan to the map
     */
void ndt_feature::NDTFeatureFuserHMT::initialize(Eigen::Affine3d initPos, const pcl::PointCloud<pcl::PointXYZ> &cloudOrig, const InterestPointVec& pts, bool preLoad){

      ///Set the cloud to sensor frame with respect to base
    
      // Copy the points... need to transform them around
		pcl::PointCloud<pcl::PointXYZ> cloud(cloudOrig);
		
		assert(cloud.points.size () == cloud.width * cloud.height);

		lslgeneric::transformPointCloudInPlace(sensor_pose, cloud);
		lslgeneric::transformPointCloudInPlace(initPos, cloud);
		Tnow = initPos;

		ptsPrev = pts; // pts are always given in the sensor frame...

		// Move the features to the current pose (Tnow)
// 		std::cout << "Move interest point" << std::endl;
		ndt_feature::moveInterestPointVec(Tnow*sensor_pose, ptsPrev);
		
// 		std::cout << "Update futur map " <<__LINE__ << " " << __FILE__ << std::endl;
		featuremap.update(ptsPrev);

		map = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(params_.resolution));
// 		std::cout << "init " <<__LINE__ << " " << __FILE__ << std::endl;
		map->initialize(Tnow.translation()(0),Tnow.translation()(1),0./*Tnow.translation()(2)*/,params_.map_size_x,params_.map_size_y,params_.map_size_z);
		
		Eigen::Affine3d Tnow_sensor = Tnow*sensor_pose; // The origin from where the sensor readings occured...
		map->addPointCloud(Tnow_sensor.translation(),cloud, 0.1, 100.0, 0.1);
// 		std::cout << "compute ndt cells " <<__LINE__ << " " << __FILE__ << std::endl;
		map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, Tnow_sensor.translation(), 0.1);

		isInit = true;
		Tlast_fuse = Tnow;
		Todom = Tnow;
		
		//Add visualization point cloud
// 		pointcloud_vis = cloudOrig;
}

    /**
     *
     * Clean up this function
     */
Eigen::Affine3d ndt_feature::NDTFeatureFuserHMT::update(Eigen::Affine3d Tmotion, const pcl::PointCloud<pcl::PointXYZ> &cloudOrig, const InterestPointVec& pts, bool updateFeatureMap, bool updateNDTMap){

    if(!isInit){
	fprintf(stderr,"NDT-FuserHMT: Call Initialize first!!\n");
	return Tnow;
      }

    debug_markers_.clear();
    
      pcl::PointCloud<pcl::PointXYZ> cloud(cloudOrig);
      pcl::PointCloud<pcl::PointXYZ> cloud_orig(cloudOrig);
      
      Eigen::Vector3d map_centroid;
      map->getCentroid(map_centroid[0], map_centroid[1], map_centroid[2]);

      // Odometry 'constraints'
      ndt_feature::MotionModel2d motion(motion_params_);
      ndt_feature::Pose2d relpose(Tmotion.translation()[0],
                                 Tmotion.translation()[1],
                                 Tmotion.rotation().eulerAngles(0,1,2)[2]);

      //      ROS_ERROR_STREAM("relpose: " << relpose);
      

      ndt_feature::Pose2dCov relposecov = motion.getPose2dCov(relpose);

      //ROS_ERROR_STREAM("relposecov: " << relposecov);
      
      Eigen::Matrix3d odom_cov = relposecov.cov;
      odom_cov(2,0) = 0.; odom_cov(2,1) = 0.; odom_cov(0,2) = 0.; odom_cov(1,2) = 0.;
      odom_cov(2,2) = 0.01; // This is the height in the ndt feature vec and not rotational variance.

      Eigen::MatrixXd TmotionCov = motion.getCovMatrix6(relpose);
      TmotionCov(2,2) = 1; // 10 // z
      TmotionCov(3,3) = 1; // roll
      TmotionCov(4,4) = 1; // pitch

      std::cerr << "TmotionCov : " << TmotionCov << std::endl;

      //lslgeneric::NDTCell* ndt_odom_cell = new lslgeneric::NDTCell();
      //      ndt_odom_cell->setMean(Eigen::Vector3d(0.,0.,0.));
      //      ndt_odom_cell->setCov(odom_cov);
      //      lslgeneric::NDTCell* ndt_odom_cell_prev = new lslgeneric::NDTCell();
      //      ndt_odom_cell_prev->setMean(Tmotion.translation());
      //      ndt_odom_cell_prev->setCov(odom_cov);
      lslgeneric::NDTCell ndt_odom_cell;
      ndt_odom_cell.setMean(Eigen::Vector3d(0.,0.,0.));
      ndt_odom_cell.setCov(odom_cov);
      lslgeneric::NDTCell ndt_odom_cell_prev;
      ndt_odom_cell_prev.setMean(Tmotion.translation());
      ndt_odom_cell_prev.setCov(odom_cov);
      
      //      ROS_ERROR_STREAM("odom_cov : " << odom_cov);

      

      Todom = Todom * Tmotion; //we track this only for display purposes!

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

      //      return Tmotion; ////////////////////////////////////////////////////////////////////// ok!!!!


      Eigen::Affine3d global_rotation;
      
      double t0=0,t1=0,t2=0,t3=0,t4=0,t5=0,t6=0;
      Eigen::Affine3d Tinit_sensor_pose = Tinit*sensor_pose;
      lslgeneric::transformPointCloudInPlace(Tinit_sensor_pose, cloud);  // Cloud -> transformed in to the vehicle origin!
      
      t0 = getDoubleTime();
      ///Create global map - TODO update the comments and naming - this is not global....
      lslgeneric::SpatialIndex* ndglobal_idx = new lslgeneric::LazyGrid(params_.resolution);
      lslgeneric::NDTMap ndglobal(ndglobal_idx, true);
      
      
      if (params_.loadCentroid) {
        if (params_.globalTransf) {
          ndglobal.loadPointCloudCentroid(cloud, Tinit_sensor_pose.translation(),
                                          map_centroid, localMapSize, params_.sensor_range);
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
          Eigen::Vector3d local_centroid = ndt_feature::computeLocalCentroid(map_centroid, Tmotion_est.translation(), params_.resolution);
          //std::cout << "local_centroid : " << local_centroid << std::endl;
          ndglobal.loadPointCloudCentroid(cloud, Tinit_sensor_pose.translation(),
                                          local_centroid, localMapSize, params_.sensor_range);
        }
      }
      else {
        if (!params_.globalTransf) {
          ndglobal.guessSize(0,0,0,params_.sensor_range,params_.sensor_range,params_.map_size_z);
          // The ndglobal map is a map with a reference frame to the vehicle origin(!)
        }
        ndglobal.loadPointCloud(cloud, params_.sensor_range); 
     }
      ndglobal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
      
      if (params_.discardCells) {
        discardCell(ndglobal, cloud.front());
        discardCell(ndglobal, cloud.back());
      }

      ROS_ERROR_STREAM("ndglobal.numberOfActiveCells() : " << ndglobal.numberOfActiveCells());
      debug_markers_.push_back(ndt_visualisation::markerNDTCells(ndglobal, 1, "ndglobal"));


      // THIS IS OK!!!
      ////      return Tmotion; ////////////////////////////////////////////////////////////////////// 

      
      t1 = getDoubleTime();
      
      // The feature handling goes here...
      Correspondences matches;
      OrientedPoint2D transform;
      std::vector<std::pair<int, int> > corr;
      Eigen::Matrix3d cov;
      cov << 0.0002, 0., 0., 0., 0.0002, 0., 0., 0., 0.0001; // Feature covarance...
      
      const double score_feature = ransac_->matchSets(ptsPrev, pts, transform, matches);
      Eigen::Affine3d Tfeat_sensor; // Local SENSOR coords to map coords
      ndt_feature::convertOrientedPoint2DToEigen(transform, Tfeat_sensor);
      // Transform to vechicle coords.
      Eigen::Affine3d Tfeat = Tfeat_sensor * sensor_pose.inverse();
      
      
      // VISUALIZATION OF FEATURE MATCHING
      bool consistent_features = true;
      {
        geometry_msgs::Pose prev_pose;
        Eigen::Affine3d Torigin;
        Torigin.setIdentity();
        tf::poseEigenToMsg(Torigin, prev_pose); // PREV
        //        debug_markers_.push_back(ndt_feature::interestPointMarkersFrameId(/*prevPts*/featuremap.getMap(), prev_pose, 1, std::string("/world"))); // ptsPrev in the map coords already...
	
        geometry_msgs::Pose curr_pose;
        tf::poseEigenToMsg(Tinit*sensor_pose, curr_pose); // CURRENT
        //        debug_markers_.push_back(ndt_feature::interestPointMarkersFrameId(pts, curr_pose, 0, std::string("/world")));
	
        if (matches.size() > 0) {
          // Seems that the order is off.
          //          debug_markers_.push_back(ndt_feature::correspondenceMarkers (matches, 
          //                                                                       prev_pose,
          //                                                                       curr_pose,
          //                                                                       std::string("/world")));
          // Check that this makes sence (not to much difference compared to the odometry...
          //		checkConsistency = true;
          Eigen::Affine3d diff = (Tnow * Tmotion).inverse() * (Tfeat);
          if((diff.translation().norm() > params_.max_translation_norm/10. || 
              diff.rotation().eulerAngles(0,1,2).norm() > params_.max_rotation_norm/4.) && params_.checkConsistency){
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
      
      
      lslgeneric::NDTMap ndt_feat_prev_sensor_frame(cv_prev_sensor_frame, true);
      lslgeneric::NDTMap ndt_feat_curr_sensor_frame(cv_curr_sensor_frame, true);
      
      lslgeneric::NDTMap* ndt_feat_prev_vehicle_frame = ndt_feat_prev_sensor_frame.pseudoTransformNDTMap(sensor_pose);
      // The current frame is always in vehicle frame (to be moved Tinit in registration)-
      lslgeneric::NDTMap* ndt_feat_curr_vehicle_frame = ndt_feat_curr_sensor_frame.pseudoTransformNDTMap(sensor_pose);
      

      if (params_.useOdom) {
        // Add odometry (if enabled last)
        //ROS_INFO_STREAM("adding odometry...");
        for (int i = 0; i < 40; i++) { // Quick HACK HERE.
          addNDTCellToMap(ndt_feat_prev_vehicle_frame, &ndt_odom_cell_prev);
          addNDTCellToMap(ndt_feat_curr_vehicle_frame, &ndt_odom_cell);
          int tmp_size = corr.size(); 
          corr.push_back(std::pair<int,int>(tmp_size, tmp_size));
        }
      }
      

      lslgeneric::NDTMap* ndt_feat_prev = ndt_feat_prev_vehicle_frame->pseudoTransformNDTMap(Tnow/**Tmotion*/);
      lslgeneric::NDTMap* ndt_feat_curr = ndt_feat_curr_vehicle_frame->pseudoTransformNDTMap(Tinit);

      // Remove the covariance rotation from the odometry (done when pseudo moving)...
      if (params_.useOdom) {
        lslgeneric::CellVector *cl = dynamic_cast<lslgeneric::CellVector*>(ndt_feat_curr->getMyIndex());
        //ROS_ERROR_STREAM(" cl->size() : " << cl->size());
        //std::cerr << "cl->getCellIdx(cl->size()-1)->getCov() : " << cl->getCellIdx(cl->size()-1)->getCov() << std::endl;
        cl->getCellIdx(cl->size()-1)->setCov(odom_cov);
        //std::cerr << "cl->getCellIdx(cl->size()-1)->getCov() :.. " << cl->getCellIdx(cl->size()-1)->getCov() << std::endl;
      }
      
      // debug_markers_.push_back(ndt_visualisation::markerNDTCells(*ndt_feat_prev, 2, "feat_prev"));
      // debug_markers_.push_back(ndt_visualisation::markerNDTCells(*ndt_feat_curr, 3, "feat_curr"));
      

      t2 = getDoubleTime();
      if (!params_.useFeat && !params_.useOdom) {
        // Both odom and feature is in the same pot here
        use_odom_or_features = false;
      }
      if (!params_.useOdom && !consistent_features) { // No odometry and no consistent features -> nothing to use...
        use_odom_or_features = false;
      }

      bool match_ok = true;
      Eigen::Affine3d Tmotion_prematched = Tmotion_est;

      if (params_.fusion2d) {
        match_ok = ndt_feature::matchFusion2d(*map, ndglobal, *ndt_feat_prev, *ndt_feat_curr, corr, Tmotion_est, true, params_.useNDT, use_odom_or_features, params_.stepcontrol, params_.ITR_MAX, params_.neighbours, params_.DELTA_SCORE) || params_.fuseIncomplete;
      }
      else {
        match_ok = ndt_feature::matchFusion(*map, ndglobal, *ndt_feat_prev, *ndt_feat_curr, corr, Tmotion_est, TmotionCov,
                                           true, params_.useNDT, use_odom_or_features, params_.stepcontrol, params_.ITR_MAX, params_.neighbours, params_.DELTA_SCORE, params_.useSoftConstraints, params_.stepControlFusion, params_.useTikhonovRegularization) || params_.fuseIncomplete;
      }

      //      std::cout << "match_ok : " << match_ok << std::endl;
      if (params_.allMatchesValid) {
        match_ok = true;
      }

      std::cout << "Tmotion_est : \n" << std::endl;
      ndt_feature::printTransf(Tmotion_est);

      //-------------------------------------------------------
      // {
      // std::cout << "Tmotion_est : ";
      // lslgeneric::printTransf2d(Tmotion_est);
      // Tnow = Tmotion_est;
      // Eigen::Affine3d spose = Tnow*sensor_pose;
      // lslgeneric::transformPointCloudInPlace(spose, cloud_orig);
      // //	    map->addPointCloudMeanUpdate(spose.translation(),cloud,localMapSize, 1e5, 1250, map_size_z/2, 0.06);
      
      // if (updateNDTMap) {
      
      
      //   map->addPointCloud(spose.translation(),cloud_orig, 0.06, 25); // Here, keep the raw cloud and add it here! TODO - check the influence here...
      //   map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, spose.translation(), 0.1);
      // }
      // }

      // delete ndt_feat_prev_vehicle_frame;
      // delete ndt_feat_curr_vehicle_frame;
      // delete ndt_feat_prev;
      // delete ndt_feat_curr;


      // return Tnow; ///////////////////////////////////////////// OK

      //------------------------------------------------------


      if (match_ok) {

        // Recompute the covariance (based on the matching...)
        if (params_.computeCov)
        {
			//COVARIANCE
			
          lslgeneric::NDTMatcherD2D matcher_d2d;
            Eigen::MatrixXd matching_cov(6,6);
            matcher_d2d.covariance(*map, ndglobal, Tmotion_est, matching_cov);
            ndt_feature::Pose2dCov posecov;
            posecov.mean = ndt_feature::pose2dFromAffine3d(Tmotion_est);
            posecov.cov = ndt_feature::cov6toCov3(matching_cov);
            ndt_feature::printTransf2d(Tmotion_est);
            ndt_feature::pose2dClearDependence(posecov);
            //            debug_markers_.push_back(ndt_visualisation::markerMeanCovariance2d(posecov.mean, posecov.cov, 100., 1, -1));
            current_posecov.mean = ndt_feature::pose2dFromAffine3d(Tmotion_est);
            Eigen::Matrix3d prev_cov = current_posecov.cov;
            current_posecov.cov = prev_cov+posecov.cov; // Only works if the registration is done from local frame to global that is that Tmotion holds the complete motion and the covariance estimate is only the local one.
            //            debug_markers_.push_back(ndt_visualisation::markerMeanCovariance2d(current_posecov.mean, current_posecov.cov, 1., 2, 0));
            
//             std::cout << "COVARIANCE :O" << std::endl;
// 			exit(0);
			
        }


        t3 = getDoubleTime();

        if (!params_.globalTransf) { // Plot the nd maps in global frame...
          lslgeneric::NDTMap* ndglobal_matched = ndglobal.pseudoTransformNDTMap(Tmotion_est);
          debug_markers_.push_back(ndt_visualisation::markerNDTCells(*ndglobal_matched, 2, "ndglobal_matched"));
          delete ndglobal_matched;

          lslgeneric::NDTMap* ndglobal_prematched = ndglobal.pseudoTransformNDTMap(Tmotion_prematched);
          debug_markers_.push_back(ndt_visualisation::markerNDTCells(*ndglobal_prematched, 0, "ndglobal_prematched"));
          delete ndglobal_prematched;
        }
        // Update the motion estimation with the orientation

        Eigen::Affine3d diff = (Tmotion_est).inverse() * Tmotion;
        if((diff.translation().norm() > params_.max_translation_norm || 
            diff.rotation().eulerAngles(0,1,2).norm() > params_.max_rotation_norm) && params_.checkConsistency){
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

          Eigen::Affine3d diff_fuse = Tlast_fuse.inverse()*Tnow;
          if(diff_fuse.translation().norm() > translation_fuse_delta ||
             diff_fuse.rotation().eulerAngles(0,1,2).norm() > rotation_fuse_delta)
          {
            t4 = getDoubleTime();
            //            debug_markers_.push_back(ndt_visualisation::markerNDTCells(*map, 1));
            
            
            Tlast_fuse = Tnow;
          }
        }
      }else{
        t3 = getDoubleTime();
        Tnow = Tnow * Tmotion;
      }
      
//       std::cout << "Tmotion_est : ";
      ndt_feature::printTransf2d(Tmotion_est);
      Eigen::Affine3d spose = Tnow*sensor_pose;
      lslgeneric::transformPointCloudInPlace(spose, cloud_orig);
      //	    map->addPointCloudMeanUpdate(spose.translation(),cloud,localMapSize, 1e5, 1250, map_size_z/2, 0.06);
      
      if (updateNDTMap) {
      
      
        map->addPointCloud(spose.translation(),cloud_orig, 0.06, 25); // Here, keep the raw cloud and add it here! TODO - check the influence here...
        map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, spose.translation(), 0.1);
      }
      t5 = getDoubleTime();
      

      ////////////////////////////////////////////////////////////////////////////////
      // if (params_.visualizeLocalCloud)
      //   pointcloud_vis = cloud;
      // else
      //   pointcloud_vis = cloud_orig;
      
      ptsPrev = pts;
      // Move the features to the current pose (Tnow)
      if (updateFeatureMap) {
        ndt_feature::moveInterestPointVec(Tnow*sensor_pose, ptsPrev);
        featuremap.update(ptsPrev);
      }



      delete ndt_feat_prev_vehicle_frame;
      delete ndt_feat_curr_vehicle_frame;
      delete ndt_feat_prev;
      delete ndt_feat_curr;

      return Tnow;
}