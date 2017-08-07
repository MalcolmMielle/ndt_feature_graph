#pragma once

//#include <ndt_feature/interfaces.h>
#include <ndt_feature/ndt_feature_link.h>
#include <ndt_feature/ndt_feature_node.h>
//#include <ndt_feature/serialization.h>
#include <ndt_map/NDTMapMsg.h>
#include <ndt_map/ndt_conversions.h>
#include <ndt_map/ndt_map.h>



namespace ndt_feature {

// This class has the same interface as the fuser class but also has an optimization function.
// initialize (first step, called once)
// for each new reading call
// update()
// whenever the optimization of the map should be done call optimize
class NDTFeatureGraph : public NDTFeatureGraphInterface {

public:

  class Params {
  public:
    Params() {
		newNodeNumberOfFrames = 20;
      newNodeTranslDist = 1.;
      storePtsInNodes = false;
      storePtsInNodesIncr = 8;
      popNodes = false;
    }
    
    void print(){
		std::cout << "newNodeNumberOfFrames = " << newNodeNumberOfFrames <<
		" newNodeTranslDist = " << newNodeTranslDist << "\n" <<
		" storePtsInNodes = " << storePtsInNodes << "\n" <<
		" storePtsInNodesIncr = " << storePtsInNodesIncr << "\n" <<
		" popNodes = " << popNodes << "\n" << std::endl;
	}
    int newNodeNumberOfFrames;
    double newNodeTranslDist;
    bool storePtsInNodes;
    int storePtsInNodesIncr;
    bool popNodes;
    
    friend std::ostream& operator<<(std::ostream &os, const NDTFeatureGraph::Params &obj)
    {
      os << "\nnewNodeTranslDist  : " << obj.newNodeTranslDist;
      os << "\nstorePtsInNodes    : " << obj.storePtsInNodes;
      os << "\nstorePtsInNodesIncr: " << obj.storePtsInNodesIncr;
      os << "\npopNodes           : " << obj.popNodes;
      return os;
    }
    
  };
  

  // Interfaces.
  virtual size_t getNbNodes() const { return nodes_.size(); }
  virtual NDTFeatureNodeInterface& getNodeInterface(size_t idx) { return nodes_[idx]; }
  virtual const NDTFeatureNodeInterface& getNodeInterface(size_t idx) const { return nodes_[idx]; }

  virtual size_t getNbLinks() const { return links_.size(); }
  virtual NDTFeatureLinkInterface& getLinkInterface(size_t idx) { return links_[idx]; }
  virtual const NDTFeatureLinkInterface& getLinkInterface(size_t idx) const { return links_[idx]; }


  NDTFeatureGraph() : distance_moved_in_last_node_(0.), count_frames_(0) {
    
  }
  
  NDTFeatureGraph(const NDTFeatureGraph::Params &params, const NDTFeatureFuserHMT::Params &fuserParams) {
    params_ = params;
    fuser_params_ = fuserParams;
  }
  
  virtual ~NDTFeatureGraph()
  {
    std::cerr << "NDTFeatureGraph Destructor" << std::endl;
     std::vector<NDTFeatureNode>::iterator it;
     for (it = nodes_.begin(); it != nodes_.end(); ++it) {
       if (it->map != NULL)
         delete it->map;
     }
     //std::for_each(nodes_.begin(), nodes_.end(), [](NDTFeatureNode &n){ delete n.map; });
    std::cerr << "NDTFeatureGraph Destructor - done" << std::endl;
  }
  
  lslgeneric::NDTMap *map;  /// The complete NDT map (will be updated when optimize is called)
  
  
	bool fullInit();
	
	
  
  // Initialize the first entry of the map.
  void initialize(Eigen::Affine3d initPose, pcl::PointCloud<pcl::PointXYZ> &cloud, const InterestPointVec& pts, bool preLoad=false);
  
   // Update the map with new readings, return the current pose in global coordinates.
  Eigen::Affine3d update(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud, const InterestPointVec& pts);

  Eigen::Affine3d incrementalPoseOffset(size_t idx_ref, size_t idx_mov);
  
  NDTFeatureLink computeLink(size_t idx_ref, size_t idx_mov);

  
  std::vector<NDTFeatureLink> getIncrementalLinks() const;
  
  
  std::vector<NDTFeatureLink> getOdometryLinks() const ;

  void updateAllGraphLinksUsingNDTRegistration(int nb_neighbours, bool keepScore);
  
  void updateLinkUsingNDTRegistration(NDTFeatureLink &link, int nb_neighbours, bool keepScore);

  void updateLinksUsingNDTRegistration(std::vector<NDTFeatureLink> &links, int nb_neighbours, bool keepScore);


  std::vector<NDTFeatureLink> getAllIncrementalFuseLinks() const;

    std::vector<NDTFeatureLink> getAllIncrementalOdomLinks() const;

  std::vector<Eigen::Affine3d> getNodePosesFromOdometry() const;

  std::vector<NDTFeatureLink> computeAllPossibleLinks();

  void clearAllLinks() {
    links_.clear();
  }
  
  void setLinks(const std::vector<ndt_feature::NDTFeatureLink> &links) {
    links_ = links;
  }

  void appendLinks(const std::vector<ndt_feature::NDTFeatureLink> &links) {
    links_.insert(links_.end(), links.begin(), links.end());
  }

  // Optimize the map.
  void optimize() {
    // For now do nothing, for the future updates nodes_.node.T.

    // Formulate a graph optimizaion problem...
    

  }

  // Fuse the nodes, that is decreasing the number of nodes_ by combining them. Note that this doesn't have to be to make them larger but simply to avoid that the number of maps grows when the robot is "re-visiting" areas.
  void fuse() {
    // For now do nothing    
  }

  void setFuserParams(const NDTFeatureFuserHMT::Params &params) {
    fuser_params_ = params;
  }

  void setMotionParams(const ndt_feature::MotionModel2d::Params &params) {
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
  
  bool saveMap();
  
  bool load(const std::string &fileName, int nb_nodes = -1);

  bool saveLinkFile(const std::string &fileName) const;

   bool loadLinkFile(const std::string &fileName);

  void setFeatureDistFunction(const HistogramDistance<double>* distFunction);
  
  NDTFeatureFuserHMT* getLastFeatureFuser() {
    return nodes_.back().map;
  }
  
  lslgeneric::NDTMap* getMap() {
    return nodes_.back().map->map;
  }
  
  lslgeneric::NDTMap* getMap(int i) {
    return nodes_[i].map->map;
  }
  
  const lslgeneric::NDTMap* getMap(int i) const {
    return nodes_[i].map->map;
  }
  
  Eigen::Affine3d getT() {
    return nodes_.back().T;
  }
  
  pcl::PointCloud<pcl::PointXYZ>& getVisualizationCloud();

  
  NDTFeatureNode& getNode(size_t idx) {
    return nodes_[idx];
  }

  const NDTFeatureNode& getNode(size_t idx) const {
    return nodes_[idx];
  }
  
  void push_back(const NDTFeatureNode& n){
	  nodes_.push_back(n);
  }
  void push_back(const NDTFeatureLink& n){
	  links_.push_back(n);
  }
  
  NDTFeatureLink& getLink(size_t idx) {
    return links_[idx];
  }

  const NDTFeatureLink& getLink(size_t idx) const {
    return links_[idx];
  }
  
  float getDistanceTravelled() const {
	  return distance_moved_in_last_node_;
  }
  
  void setDistanceTravelled(float f){
	  distance_moved_in_last_node_ = f;
  }

  void force2D();

  std::vector<NDTFeatureLink> getValidLinks(const std::vector<NDTFeatureLink> &links, double maxScore, double maxDist, double maxAngularDist, int minIdxDist) const;


  const std::vector<NDTFeatureLink>& getCurrentLinks() {
    return links_;
  }
  
  
	ndt_map::NDTMapMsg& getMapMessage();
  

  NDTViz *viewer;
  
  NDTFeatureGraph::Params params_;
  NDTFeatureFuserHMT::Params fuser_params_;
  ndt_feature::MotionModel2d::Params motion_params_;
  Eigen::Affine3d sensor_pose_;

  Eigen::Affine3d Tnow; // Current pose estimate.

  pcl::PointCloud<pcl::PointXYZ> pointcloud_vis;


protected:
  // A set of ndt_feature_fuser maps are utilized, they are encoded into graph nodes.
  std::vector<NDTFeatureNode> nodes_;
  std::vector<NDTFeatureLink> links_;


  double distance_moved_in_last_node_;
  int count_frames_;

  // set of visualization markers (the possiblity to draw a NDT set of cells for each map).

};



} // namespace
