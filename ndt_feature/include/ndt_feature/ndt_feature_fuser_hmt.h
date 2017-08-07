#ifndef NDT_FEATURE_FUSER_HMT_HH
#define NDT_FEATURE_FUSER_HMT_HH

#include <ndt_visualisation/ndt_viz.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_map_hmt.h>
#include <ndt_registration/ndt_matcher_d2d_2d.h>
#include <ndt_registration/ndt_matcher_d2d.h>
#include <ndt_feature/ndt_matcher_d2d_fusion.h>
#include <eigen_conversions/eigen_msg.h>
// #include <ndt_feature/motion_model2d.h>

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
#include <ndt_feature/ndt_feature_map.h>
#include <ndt_feature/motion_model.hpp>

namespace ndt_feature {	
  /**
   * \brief This class fuses new point clouds into a common ndt map reference, keeping tack of the 
   * camera postion.
   * \author Jari, Todor
   */
class NDTFeatureFuserHMT{
  public:
    Eigen::Affine3d Tnow, Tlast_fuse, Todom; ///< current pose
    lslgeneric::NDTMap *map;  ///< da map
  //    std::vector< std::vector<InterestPoint *> > featuremap;
  NDTFeatureMap featuremap;


    NDTViz *viewer;
    FILE *fAddTimes, *fRegTimes;

    boost::shared_ptr<RansacFeatureSetMatcher> ransac_;

  InterestPointVec ptsPrev;
  int ctr;
  
  // Containers used for visualizations.
  std::vector<visualization_msgs::Marker> debug_markers_;
  pcl::PointCloud<pcl::PointXYZ> pointcloud_vis;
  
  
  //! Parameters of the NDT - feature - class, note some parameters are free to be altered during execution. Check the initalization function.
  class Params {
  public:
    Params() {
      // old params
      checkConsistency = false;		  ///perform a check for consistency against initial estimate
      resolution = 1.;
      map_size_x = 40.;
      map_size_y = 40.;
      map_size_z = 10.;
      sensor_range = 3.;
      max_translation_norm = 1;
      max_rotation_norm = M_PI/4.;
      fuseIncomplete = false;
      beHMT = true;
      prefix = "";
      hmt_map_dir = "map";
      
      
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
      discardCells = false;
      useSoftConstraints = true;
      computeCov = true;
      stepControlFusion = true;
      useTikhonovRegularization = true;
    }
    
    void print(){
		// old params
       std::cout<< " checkConsistency = " << checkConsistency << "\n" <<		  ///perform a check for consistency against initial estimate
      " resolution = " << resolution << "\n" <<
      " map_size_x = " << map_size_x << "\n" <<
      " map_size_y = " << map_size_y << "\n" <<
      " map_size_z = " << map_size_z << "\n" <<
      " sensor_range = " << sensor_range << "\n" <<
      " max_translation_norm = " << max_translation_norm << "\n" <<
      " max_rotation_norm = " << max_rotation_norm << "\n" <<
      " fuseIncomplete = " << fuseIncomplete << "\n" <<
      " beHMT = " << beHMT << "\n" <<
      " prefix = " << prefix << "\n" <<
      " hmt_map_dir = " << hmt_map_dir << "\n" <<
      
      
      // new params
      " useNDT = " << useNDT << "\n" <<
      " useFeat = " << useFeat << "\n" <<
      " useOdom = " << useOdom << "\n" <<
      " neighbours = " << neighbours << "\n" <<
      " stepcontrol = " << stepcontrol << "\n" <<
      " ITR_MAX = " << ITR_MAX << "\n" <<
      " DELTA_SCORE = " << DELTA_SCORE << "\n" <<
      " globalTransf = " << globalTransf << "\n" <<
      " loadCentroid = " << loadCentroid << "\n" <<
      " forceOdomAsEst = " << forceOdomAsEst << "\n" <<
      " visualizeLocalCloud = " << visualizeLocalCloud << "\n" <<
      " fusion2d = " << fusion2d << "\n" <<
      " allMatchesValid = " << allMatchesValid << "\n" <<
      " discardCells = " << discardCells << "\n" <<
//       " optimizeOnlyYaw = " << optimizeOnlyYaw << "\n" <<
      " computeCov = " << computeCov << "\n" << std::endl;
	}
    
    bool checkConsistency;
    double resolution;
    double map_size_x;
    double map_size_y;
    double map_size_z;
    double sensor_range;
    double max_translation_norm;
    double max_rotation_norm;
    bool fuseIncomplete;
    bool beHMT;
    std::string prefix;
    std::string hmt_map_dir;
       
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
    bool discardCells;
    bool useSoftConstraints;
    bool computeCov;
    bool stepControlFusion;
    bool useTikhonovRegularization;
  
    std::string getDescString() const {
      std::ostringstream os;
      //      os << "useNDT" << useNDT << "useFeat" << useFeat << "useOdom" << useOdom << "loadCentroid" << loadCentroid << "discardCells" << discardCells << "neighbours" << neighbours;
      os << "resolution" << resolution << "loadCentroid" << loadCentroid << "discardCells" << discardCells << "neighbours" << neighbours << "forceOdomAsEst" << forceOdomAsEst << "useSoftConstraints" << useSoftConstraints;
      return os.str();
    }
    
    friend std::ostream& operator<<(std::ostream &os, const NDTFeatureFuserHMT::Params &obj)
    {
      os << "\nresolution           : " << obj.resolution;
      os << "\ncheckConsistency     : " << obj.checkConsistency;
      os << "\nmap_size_x           : " << obj.map_size_x;
      os << "\nmap_size_y           : " << obj.map_size_y;
      os << "\nmap_size_z           : " << obj.map_size_z;
      os << "\nsensor_range         : " << obj.sensor_range;
      os << "\nmax_translation_norm : " << obj.max_translation_norm;
      os << "\nmax_rotation_norm    : " << obj.max_rotation_norm;
      os << "\nfuseIncomplete       : " << obj.fuseIncomplete;
      os << "\nbeHMT                : " << obj.beHMT;
      os << "\nprefix               : " << obj.prefix;
      os << "\nhmt_map_dir          : " << obj.hmt_map_dir;
      

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
      os << "\ndiscardCells: " << obj.discardCells;
      os << "\nuseSoftConstraints : " << obj.useSoftConstraints;
      os << "\ncomputeCov  : " << obj.computeCov;
      os << "\nstepControlFusion : " << obj.stepControlFusion;
      os << "\nuseTikhonovRegularization : " << obj.useTikhonovRegularization;
      return os;
    }
  };
  
  Params params_;
  ndt_feature::MotionModel2d::Params motion_params_;

  NDTFeatureFuserHMT(const NDTFeatureFuserHMT::Params &params) : 
    ransac_(new RansacFeatureSetMatcher(0.0599, 0.9, 0.1, 0.6, 0.0499, false)), params_(params), viewer(NULL), map(NULL) {

    //    ROS_INFO_STREAM("NDTFeatureFuserHMT - constructor");
    isInit = false;

    visualize = false; // 
    
    //Init of those by Malcolm
    Tnow.setIdentity();
// 	std::cout << "Doing the TNOW" << Tnow.matrix() <<std::endl;
// 	exit(0);
	Tlast_fuse.setIdentity(); 
	Todom.setIdentity();

    sensor_pose.setIdentity();
    translation_fuse_delta = 0.05;
    rotation_fuse_delta = 0.01;
    ctr =0;

    localMapSize<<params.sensor_range+3*params.resolution,params.sensor_range+3*params.resolution,params.map_size_z;
    
    // char fname[1000];
    // snprintf(fname,999,"%s_addTime.txt",params.prefix.c_str());
    // fAddTimes = fopen(fname,"w");
    
    std::cout<<"MAP: resolution: "<<params.resolution<<" size "<<params.map_size_x<<" "<<params.map_size_y<<" "<<params.map_size_z<<" sr "<<params.sensor_range<<std::endl;
  }

  
  

    ~NDTFeatureFuserHMT()
      {
        std::cerr << "NDTFeatureFuserHMT Destructor() " << std::endl;
        if (viewer!=NULL) 
          delete viewer;
	// if(fAddTimes!=NULL) fclose(fAddTimes);
	// if(fRegTimes!=NULL) fclose(fRegTimes);
        if (map != NULL) 
          delete map;
        
        std::cerr << "NDTFeatureFuserHMT Destructor() - done " << std::endl;
      }

  void setMotionParams(const ndt_feature::MotionModel2d::Params &params) {
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

    bool saveMap();
  

  bool save(const std::string &fileName);

  bool load(const std::string &fileName);


  // const std::vector<visualization_msgs::Marker>& getVisualizationMarkers() const {
  //   return debug_markers_;
  // }

  pcl::PointCloud<pcl::PointXYZ>& getVisualizationCloud() ;

  

    /**
     * Set the initial position and set the first scan to the map
     */
  void initialize(Eigen::Affine3d initPos, const pcl::PointCloud<pcl::PointXYZ> &cloudOrig, const InterestPointVec& pts, bool preLoad=false);

    /**
     *
     * Clean up this function
     */
  Eigen::Affine3d update(Eigen::Affine3d Tmotion, const pcl::PointCloud<pcl::PointXYZ> &cloudOrig, const InterestPointVec& pts, bool updateFeatureMap = true, bool updateNDTMap = true);

  NDTFeatureMap& getFeatureMap() {
    return featuremap;
  }

  const NDTFeatureMap& getFeatureMap() const {
    return featuremap;
  }
  
  const NDTFeatureFuserHMT::Params& getParam() const {
	  return params_;
  }

  
  Eigen::Matrix3d& getCov(){
	  return current_posecov.cov;
  }
  
  private:
    bool isInit;
    double translation_fuse_delta, rotation_fuse_delta;
    bool visualize;

    Eigen::Affine3d sensor_pose;
    Eigen::Vector3d localMapSize;

  ndt_feature::Pose2dCov current_posecov;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      };
} // namespace

// // Forward declaration of class boost::serialization::access
// namespace boost {
//   namespace serialization {
//     class access;
//     
//     // Serialization of Eigen::Vector2d
//     template<typename Archive>
//     inline void serialize(Archive& ar, ndt_feature::NDTFeatureFuserHMT::Params& o, const unsigned int version) {
//       ar & o.checkConsistency 
//         & o.resolution
//         & o.map_size_x
//         & o.map_size_y
//         & o.map_size_z
//         & o.sensor_range
//         & o.max_translation_norm
//         & o.max_rotation_norm
//         & o.fuseIncomplete
//         & o.beHMT
//         & o.prefix
//         & o.hmt_map_dir
//         & o.useNDT
//         & o.useFeat
//         & o.useOdom
//         & o.neighbours
//         & o.stepcontrol
//         & o.ITR_MAX
//         & o.DELTA_SCORE
//         & o.globalTransf
//         & o.loadCentroid
//         & o.forceOdomAsEst
//         & o.visualizeLocalCloud
//         & o.fusion2d
//         & o.allMatchesValid
//         & o.discardCells
//         & o.useSoftConstraints;
//     }
//   }
// }
  

#endif
