#pragma once

#include <ndt_feature/ndt_feature_node.h>

namespace ndt_feature {

class NDTFeatureNodeLink {
public:
  NDTFeatureNodeLink(size_t ref, size_t mov) : ref_idx(ref), mov_idx(mov) { }
  size_t ref_idx; // Vector idx...
  size_t mov_idx;
  Eigen::Affine3d T; // From ref->mov.
  Eigen::MatrixXd cov;
  double score;

  friend std::ostream& operator<<(std::ostream &os, const NDTFeatureNodeLink &obj)
  {
    os << "\n[" << obj.ref_idx << "<->" << obj.mov_idx << "] T: " << "["<<obj.T.translation()[0] << "," 
            << obj.T.translation()[1] << "]("
       << obj.T.rotation().eulerAngles(0,1,2)[2] << ")";
    os << "\n score : " << obj.score;
    return os;
    }
  
};

// This class has the same interface as the fuser class but also has an optimization function.
// initialize (first step, called once)
// for each new reading call
// update()
// whenever the optimization of the map should be done call optimize
class NDTFeatureGraph {

public:

  class Params {
  public:
    Params() {
    }
    double newNodeTranslDist;
    
    friend std::ostream& operator<<(std::ostream &os, const NDTFeatureGraph::Params &obj)
    {
      os << "\nnewNodeTranslDist : " << obj.newNodeTranslDist << std::endl;
      return os;
    }
    
  };
  
  NDTFeatureGraph() : distance_moved_in_last_node_(0.) {
    
  }
  
  NDTFeatureGraph(const NDTFeatureGraph::Params &params, const NDTFeatureFuserHMT::Params &fuserParams) {
    params_ = params;
    fuser_params_ = fuserParams;
  }
  
  ~NDTFeatureGraph()
  {
    std::vector<NDTFeatureNode>::iterator it;
    for (it = nodes_.begin(); it != nodes_.end(); ++it) {
      delete it->map;
    }
    //    std::for_each(nodes_.begin(), nodes_.end(), [](NDTFeatureNode &n){ delete n.map; });
  }
  
  
  lslgeneric::NDTMap *map;  /// The complete NDT map (will be updated when optimize is called)
  
  // Initialize the first entry of the map.
  void initialize(Eigen::Affine3d initPose, pcl::PointCloud<pcl::PointXYZ> &cloud, const InterestPointVec& pts, bool preLoad=false) {
    
    std::cout << "---initialize---" << std::endl;
    // Create first entry... 
    NDTFeatureNode node;
    node.map = new NDTFeatureFuserHMT(fuser_params_);
    node.T = initPose;
    node.map->setMotionParams(motion_params_);
    node.map->setSensorPose(sensor_pose_);
    // Keep each map in it's own ref frame (that is node.T)
    initPose.setIdentity();
    node.map->initialize(initPose, cloud, pts, preLoad);
    nodes_.push_back(node);
    std::cout << "initialize -> done" << std::endl;
  }

  // Update the map with new readings, return the current pose in global coordinates.
  Eigen::Affine3d update(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud, const InterestPointVec& pts){

    std::cout << "---update---" << std::endl;
    NDTFeatureNode &node = nodes_.back();
    distance_moved_in_last_node_ += Tmotion.translation().norm();
    
    // Check if we should start on a new map?
    if (distance_moved_in_last_node_ > params_.newNodeTranslDist) {
      distance_moved_in_last_node_ = 0.;
      // The returned pose is the local map coord
      // Do not update the feature map in this step.
      
      Eigen::Affine3d Tnow_local = node.map->update(Tmotion, cloud, pts, false, false);
      Tnow = node.T*Tnow_local;
      
      // New map (one option is to add the last reading to the previous map and then create a new map with the first reading only). This to make sure that the initial pose estimate of the map is using as much information as possible and that not only odometry readings are used. Possible this could be done without adding the readings to the current active map.
      

      // Create a new map.
      NDTFeatureNode new_node;
      new_node.map = new NDTFeatureFuserHMT(fuser_params_);
      new_node.map->setMotionParams(motion_params_);
      new_node.map->setSensorPose(sensor_pose_);
      new_node.T = Tnow;
      // Add the first data...
      Eigen::Affine3d init_pose;
      init_pose.setIdentity();
      new_node.map->initialize(init_pose, cloud, pts, false);
      nodes_.push_back(new_node);
      std::cout << "update -> done" << std::endl;
      return Tnow;
    }
    else {
      // The returned pose is the local map coord.
      Eigen::Affine3d Tnow_local = node.map->update(Tmotion, cloud, pts);
      Tnow = node.T*Tnow_local;
    }
    std::cout << "nodes_.size() : " << nodes_.size() << std::endl;
    std::cout << "update -> done" << std::endl;
    return Tnow;
  }

  Eigen::Affine3d incrementalPoseOffset(size_t idx_ref, size_t idx_mov) {
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

  NDTFeatureNodeLink computeLink(size_t idx_ref, size_t idx_mov) {
    NDTFeatureNodeLink m(idx_ref, idx_mov);
    Correspondences matches;
    
    std::cout << "computeLink : " << idx_ref << ", " << idx_mov << std::endl;

    double feature_score = matchNodesUsingFeatureMap(nodes_[idx_ref], nodes_[idx_mov], matches, m.T);
    //m.score = feature_score;
    // Note that the feature_score value is barely useful.
    // Use the occupancy overlap value instead...
    m.score = overlapNDTOccupancyScore(nodes_[idx_ref], nodes_[idx_mov], m.T);
    return m;
  }

  std::vector<NDTFeatureNodeLink> computeAllPossibleLinks() {
    std::vector<NDTFeatureNodeLink> ret;
    // Step through the graph and find links between nodes...
    size_t i = 0; 
    for (size_t i = 0; i < nodes_.size(); i++) {
      for (size_t j = i+1; j < nodes_.size(); j++) {
        ret.push_back(computeLink(i, j));
      }
    }
    return ret;
  }
  
  // Optimize the map.
  void optimize() {
    // For now do nothing, for the future updates nodes_.node.T.
  }

  // Fuse the nodes, that is decreasing the number of nodes_ by combining them. Note that this doesn't have to be to make them larger but simply to avoid that the number of maps grows when the robot is "re-visiting" areas.
  void fuse() {
    // For now do nothing    
  }

  void setFuserParams(const NDTFeatureFuserHMT::Params &params) {
    fuser_params_ = params;
  }

  void setMotionParams(const semrob_generic::MotionModel2d::Params &params) {
    motion_params_ = params;
  }
  
  void setSensorPose(Eigen::Affine3d spose){
    sensor_pose_ = spose;
  }

  bool wasInit() const {
    // Fuser function...
    if (nodes_.empty())
      return false;
    return nodes_.front().map->wasInit();
  }
  
  bool saveMap() {
    std::string filename = fuser_params_.hmt_map_dir + fuser_params_.prefix;
    // Save some graph based params
    
    // Note - since there is different file formats used here, each node is saved to a separate file instead of having one larger file, one file is used for the high level params.
    std::ofstream ofs(filename.c_str());
    boost::archive::text_oarchive ar(ofs);
    size_t s = nodes_.size();
    ar & s;
    ar & fuser_params_;
    ar & motion_params_;
    ar & sensor_pose_;

    ofs.close();
    
    std::vector<NDTFeatureNode>::iterator it;
    int i = 0; 
    for (it = nodes_.begin(); it != nodes_.end(); it++) {
      it->save(filename + semrob_generic::toString(i));
      i++;
    }
    return true;
  }
  
  bool load(const std::string &fileName, int nb_nodes = -1) {
    bool ret = true;
    std::ifstream ifs(fileName.c_str());
    boost::archive::text_iarchive ar(ifs);
    size_t s;
    ar & s;
    ar & fuser_params_;
    ar & motion_params_;
    ar & sensor_pose_;
    ifs.close();
    
    std::string filename = fuser_params_.hmt_map_dir + fuser_params_.prefix;
    if (nb_nodes > 0)
      s = nb_nodes;
    
    for (size_t i = 0; i < s; i++) {
      NDTFeatureNode new_node;
      new_node.map = new NDTFeatureFuserHMT(fuser_params_);
      new_node.map->setMotionParams(motion_params_);
      new_node.map->setSensorPose(sensor_pose_);
      std::string str = filename + semrob_generic::toString(i);
      if (!new_node.load(str)) {
        ret = false;
        std::cerr << "failed to load : " << str << std::endl;
      }
      nodes_.push_back(new_node);
    }
    return ret;
  }

  void setFeatureDistFunction(const HistogramDistance<double>* distFunction) {
    std::vector<NDTFeatureNode>::iterator it;
    for (it = nodes_.begin(); it != nodes_.end(); ++it) {
      it->getFeatureMap().setDistFunction(distFunction);
    }
  }
  
  NDTFeatureFuserHMT* getLastFeatureFuser() {
    return nodes_.back().map;
  }
  
  lslgeneric::NDTMap* getMap() {
    return nodes_.back().map->map;
  }
  
  Eigen::Affine3d getT() {
    return nodes_.back().T;
  }
  
  pcl::PointCloud<pcl::PointXYZ>& getVisualizationCloud() {
    if (wasInit()) {
      // Rotate / translate this with current T value.
      pcl::PointCloud<pcl::PointXYZ> &pc = nodes_.back().map->pointcloud_vis;
      lslgeneric::transformPointCloudInPlace(nodes_.back().T, pc);
      return pc;
    }
    return pointcloud_vis;
  }

  
  
  NDTViz *viewer;
  
  NDTFeatureGraph::Params params_;
  NDTFeatureFuserHMT::Params fuser_params_;
  semrob_generic::MotionModel2d::Params motion_params_;
  Eigen::Affine3d sensor_pose_;

  Eigen::Affine3d Tnow; // Current pose estimate.

  pcl::PointCloud<pcl::PointXYZ> pointcloud_vis;


  NDTFeatureNode& getNode(size_t idx) {
    return nodes_[idx];
  }

  size_t getNbNodes() const { return nodes_.size(); }

private:
  // A set of ndt_feature_fuser maps are utilized, they are encoded into graph nodes.
  std::vector<NDTFeatureNode> nodes_;


  double distance_moved_in_last_node_;

  // set of visualization markers (the possiblity to draw a NDT set of cells for each map).

};




} // namespace
