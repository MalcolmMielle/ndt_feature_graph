#include "ndt_feature/ndt_feature_graph.h"



// This class has the same interface as the fuser class but also has an optimization function.
// initialize (first step, called once)
// for each new reading call
// update()
// whenever the optimization of the map should be done call optimize

bool ndt_feature::NDTFeatureGraph::fullInit()
{
	for(size_t i = 0 ; i < getNbNodes() ; ++i){
		if(nodes_[i].map->wasInit() == false){
			std::cout << "False on the " << i + 1 << " element out of " << getNbNodes() << " elements." << std::endl;
			return false;
		}
	}
	return true;
}

  
  // Initialize the first entry of the map.
  void ndt_feature::NDTFeatureGraph::initialize(Eigen::Affine3d initPose, pcl::PointCloud<pcl::PointXYZ> &cloud, const InterestPointVec& pts, bool preLoad) {
    
    std::cout << "---initialize---" << std::endl;
    // Create first entry... 
    NDTFeatureNode node;
    node.map = new NDTFeatureFuserHMT(fuser_params_);
    node.T = initPose;
    node.map->setMotionParams(motion_params_);
    node.map->setSensorPose(sensor_pose_);
    // Keep each map in it's own ref frame (that is node.T)
    initPose.setIdentity();
// 	std::cout << "Initializing node " << std::endl;
    node.map->initialize(initPose, cloud, pts, preLoad);

    // Add the cloud...
    if (params_.storePtsInNodes) {
// 		std::cout << "STORING THE POINT CLOUD" << std::endl;
		Eigen::Affine3d Tnow_local_sensor = initPose*sensor_pose_;
		node.addCloud(Tnow_local_sensor, cloud);
    }

    nodes_.push_back(node);
	
	assert(node.map->wasInit() == true);
	assert(nodes_[0].map->wasInit() == true);
	
	if(fullInit() == false){
// 		std::cout << "GRAPH NOT FULLY INIT " << std::endl;
		assert(fullInit());
	}
    std::cout << "initialize -> done" << std::endl;
  }
  
 

  // Update the map with new readings, return the current pose in global coordinates.
  Eigen::Affine3d ndt_feature::NDTFeatureGraph::update(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud, const InterestPointVec& pts){

	  std::cout << "Graph update. Nb nof nodes : " << getNbNodes() << std::endl;
    NDTFeatureNode &node = nodes_.back();
    std::cout << "---update--- # " << node.nbUpdates << " node # " << nodes_.size() << std::endl;
    std::cout << " odom : " << std::flush; ndt_feature::printTransf2d(node.Tlocal_odom);

	assert(node.map->wasInit() == true);

    distance_moved_in_last_node_ += Tmotion.translation().norm();
    
    // Check if we should start on a new map?
    if (distance_moved_in_last_node_ > params_.newNodeTranslDist) {
      distance_moved_in_last_node_ = 0.;
      // The returned pose is the local map coord
      // Do not update the feature map in this step.
      
// 	  std::cout << "Updating node " << std::endl;
      Eigen::Affine3d Tnow_local = node.map->update(Tmotion, cloud, pts, false, false);
      Tnow = node.T*Tnow_local;
      node.Tlocal_odom = node.Tlocal_odom*Tmotion;
      node.Tlocal_fuse = Tnow_local;
      
      // New map (one option is to add the last reading to the previous map and then create a new map with the first reading only). This to make sure that the initial pose estimate of the map is using as much information as possible and that not only odometry readings are used. Possible this could be done without adding the readings to the current active map.
      

      // Create a new map.
      NDTFeatureNode new_node;
      new_node.map = new NDTFeatureFuserHMT(fuser_params_);
      new_node.map->setMotionParams(motion_params_);
      new_node.map->setSensorPose(sensor_pose_);
      new_node.T = Tnow;
	  
	  
// 	  new_node.Tlocal_odom = node.Tlocal_odom*Tmotion;
//       new_node.Tlocal_fuse = Tnow_local;
      
	  // Add the first data...
      Eigen::Affine3d init_pose;
      init_pose.setIdentity();
// 	  std::cout << "Creating new_node init " << std::endl;
      new_node.map->initialize(init_pose, cloud, pts, false);
// 	  std::cout << "Done new_node init " << std::endl;

      // Add the cloud...
//       if (params_.storePtsInNodes) {
//         Eigen::Affine3d Tnow_local_sensor = init_pose*sensor_pose_;
//         new_node.addCloud(Tnow_local_sensor, cloud);
//       }
      
      if (params_.popNodes) {
        std::cerr << "POP NODES!!!-------------------" << std::endl;
        nodes_.pop_back();
      }
      nodes_.push_back(new_node);
      std::cout << "update -> done" << std::endl;
      return Tnow;
    }
    
    
// 	std::cout << "Update the node without creating any new node ?" << std::endl;

    // The returned pose is the local map coord.
	assert(node.map->wasInit() == true);
    Eigen::Affine3d Tnow_local = node.map->update(Tmotion, cloud, pts);
// 	std::cout << "Updated the node without creating any new node" << std::endl;

    Tnow = node.T*Tnow_local;
    node.Tlocal_odom = node.Tlocal_odom*Tmotion;
    node.Tlocal_fuse = Tnow_local;
    
    node.nbUpdates++;

    // Add the cloud...
    if (params_.storePtsInNodes) {
      if (node.nbUpdates % params_.storePtsInNodesIncr == 0) {
        Eigen::Affine3d Tnow_local_sensor = Tnow_local*sensor_pose_;
        node.addCloud(Tnow_local_sensor, cloud);
      }
    }

    std::cout << "nodes_.size() : " << nodes_.size() << std::endl;
    std::cout << "update -> done" << std::endl;
    return Tnow;
  }

  Eigen::Affine3d ndt_feature::NDTFeatureGraph::incrementalPoseOffset(size_t idx_ref, size_t idx_mov) {
    Eigen::Affine3d T;
    T.setIdentity();
    if (idx_ref < idx_mov) {
      for (size_t i = idx_ref; i <= idx_mov; i++) {
        T = T*nodes_[i].T;
      }
    }
    else {
      for (size_t i = idx_mov; i >= idx_ref; i--) {
        T = T*nodes_[i].T.inverse();
      }
    }
    return T;
  }

  ndt_feature::NDTFeatureLink ndt_feature::NDTFeatureGraph::computeLink(size_t idx_ref, size_t idx_mov) {
    NDTFeatureLink m(idx_ref, idx_mov);
    Correspondences matches;
    
    std::cout << "computeLink : " << idx_ref << ", " << idx_mov << std::endl;

    std::cout << "#feature ref : " << nodes_[idx_ref].map->featuremap.map.size() << std::endl;
    std::cout << "#feature mov : " << nodes_[idx_mov].map->featuremap.map.size() << std::endl;

    double feature_score = matchNodesUsingFeatureMap(nodes_[idx_ref], nodes_[idx_mov], matches, m.T);
    //m.score = feature_score;
    // Note that the feature_score value is barely useful.
    // Use the occupancy overlap value instead...
    m.score = overlapNDTOccupancyScore(nodes_[idx_ref], nodes_[idx_mov], m.T);
    return m;
  }

  
  std::vector<ndt_feature::NDTFeatureLink> ndt_feature::NDTFeatureGraph::getIncrementalLinks() const {
    std::vector<NDTFeatureLink> ret;
    for (size_t i = 0; i < (int) nodes_.size() - 1 ; i++) {
      NDTFeatureLink m(i, i+1);
      //m.T = nodes_[i].T.rotation()*nodes_[i].T.inverse()*nodes_[i+1].T;;
      //m.T = Eigen::Translation<double,3>(nodes_[i+1].T.translation()-nodes_[i].T.translation())*nodes_[i+1].T.rotation();
      m.T = nodes_[i].T.inverse()*nodes_[i+1].T;

      std::cout << "---" << std::endl;
      std::cout << "incr : " << std::flush; ndt_feature::printTransf(m.T);
      std::cout << "T  : " << std::flush; ndt_feature::printTransf(nodes_[i].T);
      std::cout << "T+1  : " << std::flush; ndt_feature::printTransf(nodes_[i+1].T);
      std::cout << "T*incr : " << std::flush; ndt_feature::printTransf(nodes_[i].T*m.T);
      
      std::cout << "m.T.rotation() : " << m.T.rotation() << std::endl;
      std::cout << "m.T.rotation().eulerAngles(0,1,2) : " <<
        m.T.rotation().eulerAngles(0,1,2) << std::endl;

      m.score = -1.;
      ret.push_back(m);
    }
    return ret;
  
  }
  
  
  std::vector<ndt_feature::NDTFeatureLink> ndt_feature::NDTFeatureGraph::getOdometryLinks() const {
    std::vector<NDTFeatureLink> ret;
    for (size_t i = 0; i < (int) nodes_.size() - 1 ; i++) {
	  std::cout << "Checking out node " << std::endl;
      NDTFeatureLink m(i, i+1);
	  Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
// 	  std::cout <<"Cov on creation " <<m.getRelCov().inverse().format(cleanFmt) << std::endl;
      //m.T = nodes_[i].T.rotation()*nodes_[i].T.inverse()*nodes_[i+1].T;;
      //m.T = Eigen::Translation<double,3>(nodes_[i+1].T.translation()-nodes_[i].T.translation())*nodes_[i+1].T.rotation();
      m.T = nodes_[i].Tlocal_odom; //<-Not multiuplied since it's alwasy starts at zero again so we don't need a difference
      //.inverse()*nodes_[i+1].Tlocal_odom;

      std::cout << "---" << std::endl;
      std::cout << "incr : " << std::flush; ndt_feature::printTransf(m.T);
      std::cout << "T  : " << std::flush; ndt_feature::printTransf(nodes_[i].T);
      std::cout << "T+1  : " << std::flush; ndt_feature::printTransf(nodes_[i+1].T);
      std::cout << "T*incr : " << std::flush; ndt_feature::printTransf(nodes_[i].T*m.T);
      
      std::cout << "m.T.rotation() : " << m.T.rotation() << std::endl;
      std::cout << "m.T.rotation().eulerAngles(0,1,2) : " <<
	  m.T.rotation().eulerAngles(0,1,2) << std::endl;
      m.score = -1.;
      ret.push_back(m);
	  std::cout <<"Cov on push_back " <<m.getRelCov().inverse().format(cleanFmt) << std::endl;
	      
		
	}
	
	for(int tmp = 0 ; tmp < ret.size() ; ++tmp){
	Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
// 			std::cout <<"Estimate before return " << ret[tmp].getRelCov().inverse().format(cleanFmt) << std::endl;
		std::cout << "Adding Edge" << std::endl;	
	}
    
    
    return ret;
  
  }

  void ndt_feature::NDTFeatureGraph::updateAllGraphLinksUsingNDTRegistration(int nb_neighbours, bool keepScore){
	  
	  for(size_t i = 0 ; i < getNbLinks() ; ++i){
		  
		  std::cout << "New link" << __LINE__ << std::endl;
		  
		  NDTFeatureLink link = (NDTFeatureLink&) getLinkInterface(i);
		  updateLinkUsingNDTRegistration(link, nb_neighbours, keepScore);
		  
		  std::cout << "OUT" << __LINE__ << std::endl;
		  
	  }
	  
  }
  
  void ndt_feature::NDTFeatureGraph::updateLinkUsingNDTRegistration(NDTFeatureLink &link, int nb_neighbours, bool keepScore) {
    lslgeneric::NDTMatcherD2D matcher_d2d;
    matcher_d2d.n_neighbours = nb_neighbours;
	
	std::cout << "Matching : " << link.getRefIdx() << " with " << link.getMovIdx() << std::endl;
	Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
	std::cout << "Transform between the two " << link.T.matrix().format(cleanFmt) << std::endl; 
    int a ;
// 	std::cout << "PAUSE before match" << std::endl;
// 	std::cin >> a;
	
	Eigen::Affine3d before_T = link.T;
	
    bool converged = matcher_d2d.match(nodes_[link.getRefIdx()].getNDTMap(), nodes_[link.getMovIdx()].getNDTMap(), link.T, true);
	
	
	
	std::cout << "Transform between the two new " << link.T.matrix().format(cleanFmt) << std::endl; 
	
// 	int a ;
// 	std::cout << "PAUSE before cov" << std::endl;
// 	std::cin >> a;
	
	//Adding the covariance into the link
	Eigen::MatrixXd cov(6,6);
	
	bool same = true;
	for(size_t i = 0; i < 4 ; ++i){
		
		for(size_t  j = 0 ; j < 4 ; ++j){
			if(before_T(i,j) != link.T(i,j)){
				same = false;
			}
		}
	}
	
	if(same == false){
		cov.setZero();
		matcher_d2d.covariance(nodes_[link.getRefIdx()].getNDTMap(), nodes_[link.getMovIdx()].getNDTMap(), link.T, cov);
	}
	else{
		std::cout << "NOTHING HAPPENED Creating a identity matrix" << std::endl;
		cov = Eigen::MatrixXd::Identity(6, 6);
		cov << 	0.02, 	0, 		0,
				0, 		0.02, 	0,
				0, 		0, 		0.02;
// 		exit(0);
	}
	std::cout << "Size of Covariance : " << cov.rows() << " AND COLS " << cov.cols() << std::endl;
	
	assert(cov.rows() == 6);
	assert(cov.cols() == 6);
	std::cout << "PAUSE got cov : " << cov << "\n";
	std::cout << "COVARIANCE BY MATCHER " << cov.inverse().format(cleanFmt) << "\n"; 
	
	if(converged == false){
// 		throw std::runtime_error("ndt_map registration didn't converge");
		std::cout << "USing odometry input a number to continue" << std::endl;
		int a;
		std::cin >> a;
	}
	if(same != false){
		std::cout << "Manually created the covariance because the matching returned the same transof as before." << std::endl;
		int a;
		std::cin >> a;	
	}
// 	exit(0);
	link.cov_3d = cov;
	std::cout << "How my god" << "\n";
	
// 	std::cin >> a;
	
    if (!keepScore) {
      std::cout << "old link.score : " << "\n";
	  std::cout << link.score << "\n";
      link.score = ndt_feature::overlapNDTOccupancyScore(nodes_[link.getRefIdx()],
                                                         nodes_[link.getMovIdx()], 
                                                         link.T);
      std::cout << "new link.score : " << link.score << "\n";
    }
    
    std::cout << "End of link" << std::endl;
  }

  void ndt_feature::NDTFeatureGraph::updateLinksUsingNDTRegistration(std::vector<NDTFeatureLink> &links, int nb_neighbours, bool keepScore) {

    for (size_t i = 0; i < links.size(); i++) {
//       std::cout << "updating link : " << i << " (size of links :" << links.size() << ")" << std::endl;
      updateLinkUsingNDTRegistration(links[i], nb_neighbours, keepScore);
    }
  }


  std::vector<ndt_feature::NDTFeatureLink> ndt_feature::NDTFeatureGraph::getAllIncrementalFuseLinks() const {
    std::vector<NDTFeatureLink> ret;
    for (size_t i = 0; i < nodes_.size()-1; i++) {
      NDTFeatureLink m(i, i+1);
      m.T = nodes_[i].Tlocal_fuse;
      m.score = -1.;
      ret.push_back(m);
    }
    return ret;
  }

    std::vector<ndt_feature::NDTFeatureLink> ndt_feature::NDTFeatureGraph::getAllIncrementalOdomLinks() const {
    std::vector<NDTFeatureLink> ret;
    for (size_t i = 0; i < nodes_.size()-1; i++) {
      NDTFeatureLink m(i, i+1);
      m.T = nodes_[i].Tlocal_odom;
      m.score = -1.;
      ret.push_back(m);
    }
    return ret;
  }

std::vector<Eigen::Affine3d> ndt_feature::NDTFeatureGraph::getNodePosesFromOdometry() const {
    std::vector<Eigen::Affine3d> ret;
    if (nodes_.empty())
      return ret;
    // Return all node poses based on odometry
    std::vector<NDTFeatureLink> incr_odom = getAllIncrementalOdomLinks();
    if (incr_odom.empty())
      return ret;
    Eigen::Affine3d T = nodes_[0].T;
    ret.push_back(T);
    for (size_t i = 0; i < incr_odom.size(); i++) {
      T = T*incr_odom[i].getRelPose();
      ret.push_back(T);
    }
    return ret;
}

std::vector<ndt_feature::NDTFeatureLink> ndt_feature::NDTFeatureGraph::computeAllPossibleLinks() {
    std::vector<NDTFeatureLink> ret;
    // Step through the graph and find links between nodes...
    size_t i = 0; 
    for (size_t i = 0; i < nodes_.size(); i++) {
      for (size_t j = i+1; j < nodes_.size(); j++) {
        ret.push_back(computeLink(i, j));
      }
    }
    return ret;
}

    
bool ndt_feature::NDTFeatureGraph::saveMap() {
//     std::string filename = fuser_params_.hmt_map_dir + fuser_params_.prefix;
//     // Save some graph based params
//     
//     // Note - since there is different file formats used here, each node is saved to a separate file instead of having one larger file, one file is used for the high level params.
//     std::ofstream ofs(filename.c_str());
//     boost::archive::text_oarchive ar(ofs);
//     size_t s = nodes_.size();
//     ar & s;
//     ar & fuser_params_;
//     ar & motion_params_;
//     ar & sensor_pose_;
// 
//     ofs.close();
//     
//     std::vector<NDTFeatureNode>::iterator it;
//     int i = 0; 
// 	std::cout << "Saving : " << getNbNodes() << " nodes " << std::endl;
//     for (it = nodes_.begin(); it != nodes_.end(); it++) {
// 	  std::cout << "Saving : " << i << " nodes, in : " << filename + ndt_feature::toString(i) << std::endl;
//       it->save(filename + ndt_feature::toString(i));
//       i++;
//     }
//     return true;
}
  
bool ndt_feature::NDTFeatureGraph::load(const std::string &fileName, int nb_nodes) {
//     bool ret = true;
//     std::ifstream ifs(fileName.c_str());
//     boost::archive::text_iarchive ar(ifs);
//     size_t s;
//     ar & s;
//     ar & fuser_params_;
//     ar & motion_params_;
//     ar & sensor_pose_;
//     ifs.close();
//     
//     std::string filename = fuser_params_.hmt_map_dir + fuser_params_.prefix;
//     if (nb_nodes > 0)
//       s = nb_nodes;
//     
//     for (size_t i = 0; i < s; i++) {
//       NDTFeatureNode new_node;
//       new_node.map = new NDTFeatureFuserHMT(fuser_params_);
//       new_node.map->setMotionParams(motion_params_);
//       new_node.map->setSensorPose(sensor_pose_);
//       std::string str = filename + ndt_feature::toString(i);
//       if (!new_node.load(str)) {
//         ret = false;
//         std::cerr << "failed to load : " << str << std::endl;
//       }
//       nodes_.push_back(new_node);
//     }
// 
//     if (!loadLinkFile(".links")) {
//       std::cerr << "failed to load : " << fuser_params_.hmt_map_dir + fuser_params_.prefix + std::string(".links") << std::endl;
//     }
//     return ret;
}

bool ndt_feature::NDTFeatureGraph::saveLinkFile(const std::string &fileName) const {
//     std::string filename = fuser_params_.hmt_map_dir + fuser_params_.prefix + fileName;
//     std::ofstream ofs(filename.c_str());
//     boost::archive::text_oarchive ar(ofs);
//     ar & links_;
//     ofs.close();
//     return true;
}

bool ndt_feature::NDTFeatureGraph::loadLinkFile(const std::string &fileName) {
// 	bool ret = true;
// 	std::string filename = fuser_params_.hmt_map_dir + fuser_params_.prefix + fileName;
// 	std::cout << "loading linkfile : " << filename << std::endl;
// 	std::ifstream ifs(filename.c_str());
// 	if (ifs) {
// 		boost::archive::text_iarchive ar(ifs);
// 		ar & links_;
// 		ifs.close();
// 		return true;
// 	}
// 	return false;
}

void ndt_feature::NDTFeatureGraph::setFeatureDistFunction(const HistogramDistance<double>* distFunction) {
	std::vector<NDTFeatureNode>::iterator it;
	for (it = nodes_.begin(); it != nodes_.end(); ++it) {
		it->getFeatureMap().setDistFunction(distFunction);
	}
}

pcl::PointCloud<pcl::PointXYZ>& ndt_feature::NDTFeatureGraph::getVisualizationCloud() {
	if (wasInit()) {
		// Rotate / translate this with current T value.
	//       pcl::PointCloud<pcl::PointXYZ> &pc = nodes_.back().map->pointcloud_vis;
		//Get the point cloud from the node instead of the map
		pcl::PointCloud<pcl::PointXYZ> &pc = nodes_.back().getPts();
		//WARNING : CRASH HERE !
		std::cout << pc.points.size () << " != " << pc.width * pc.height << std::endl;
		assert(pc.points.size () == pc.width * pc.height);
	// 	  std::cout << "TRANSFORM" <<std::endl;
		lslgeneric::transformPointCloudInPlace(nodes_.back().T, pc);
	// 	  std::cout << "TRANSFORM DONE" <<std::endl;
		std::cout << pc.points.size () << " != " << pc.width * pc.height << std::endl;
		assert(pc.points.size () == pc.width * pc.height);
		return pc;
	}
	return pointcloud_vis;
}

  
void ndt_feature::NDTFeatureGraph::force2D() {
	for (size_t i = 0; i < nodes_.size(); i++) {
		nodes_[i].force2D();
	}
	for (size_t i = 0; i < links_.size(); i++) {
		links_[i].force2D();
	}
}

std::vector<ndt_feature::NDTFeatureLink> ndt_feature::NDTFeatureGraph::getValidLinks(const std::vector<NDTFeatureLink> &links, double maxScore, double maxDist, double maxAngularDist, int minIdxDist) const
{
	std::vector<NDTFeatureLink> ret;
	for (size_t i = 0; i < links.size(); i++) {
		if (links[i].getScore() > maxScore) {
		continue;
		}
		
		if (abs((int)links[i].getMovIdx() - (int)links[i].getRefIdx()) < minIdxDist) {
		continue;
		}
		std::cout << "here" << std::endl;

		const NDTFeatureNode &ref_node = nodes_[links[i].getRefIdx()];
		const NDTFeatureNode &mov_node = nodes_[links[i].getMovIdx()];
		
		Eigen::Affine3d Tlink = ref_node.T*links[i].T;
		double dist, angular_dist;
		ndt_feature::distanceBetweenAffine3d(mov_node.T, Tlink, dist, angular_dist);
		
		std::cout << "dist : " << dist << std::endl;
		std::cout << "angular_dist : " << angular_dist << std::endl;

		if (dist < maxDist && angular_dist < maxAngularDist) {
		ret.push_back(links[i]);
		}

	}
	return ret;
}


ndt_map::NDTMapMsg& ndt_feature::NDTFeatureGraph::getMapMessage(){
	//Get the last ndtMap element
	lslgeneric::NDTMap* map = getMap();
	ndt_map::NDTMapMsg mapmsg;
	
// 			lslgeneric::NDTMap* map = map->pseudoTransformNDTMap();
	bool good = lslgeneric::toMessage(map, mapmsg, "/world");
	return mapmsg;
}