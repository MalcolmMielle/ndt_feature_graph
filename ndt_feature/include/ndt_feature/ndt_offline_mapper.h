#include <ndt_feature/ndt_feature_frame.h>
#include <ndt_feature/utils.h>
#include <isam/isam.h>


namespace ndt_feature {

inline isam::Pose2d convertEigenAffine3dToIsamPose2d(const Eigen::Affine3d &a) {
    isam::Pose2d p(a.translation()(0),
		   a.translation()(1),
		   //a.rotation().eulerAngles(0,1,2)(2)
                   ndt_feature::getRobustYawFromAffine3d(a)
                   );
    return p;
  }

inline Eigen::Affine3d convertIsamPose2dToEigenAffine3d(const isam::Pose2d &p) {
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(p.t(), Eigen::Vector3d::UnitZ());
    Eigen::Translation3d v(p.x(),p.y(),0);
    Eigen::Affine3d T = v*m;
    
    return T;
  }
  
inline std::vector<size_t> getNDTFeatureFrameIndicesDistance(const Eigen::Affine3d &p, const std::vector<ndt_feature::NDTFeatureFrame> &frames, size_t currIdx, double maxDist, double maxAngularDist) {
    std::vector<size_t> indices;
    for (size_t i = 0; i < currIdx; i++) {
      double dist, angular_dist;
      ndt_feature::distanceBetweenAffine3d(p, frames[i].gt, dist, angular_dist);
      if (dist < maxDist && angular_dist < maxAngularDist) {
	indices.push_back(i);
      }
    }
    return indices;
  }

inline void optimizeGraphUsingISAM(NDTFeatureGraphInterface &graph) {
  isam::Slam slam; 

  std::vector<isam::Pose2d_Node*> pose_nodes;
    
  isam::Noise noise3 = isam::Information(100. * isam::eye(3));
  isam::Noise noise2 = isam::Information(100. * isam::eye(2));
  
  // Add the nodes...
  
  // create a first pose (a node)
  isam::Pose2d_Node* new_pose_node = new isam::Pose2d_Node();
  // add it to the graph
  slam.add_node(new_pose_node);
  // also remember it locally
  pose_nodes.push_back(new_pose_node);
  
  // create a prior measurement (a factor)
  isam::Pose2d origin(0., 0., 0.);
  origin = convertEigenAffine3dToIsamPose2d(graph.getNodeInterface(0).getPose());

  isam::Pose2d_Factor* prior = new isam::Pose2d_Factor(pose_nodes[0], origin, noise3);
  // add it to the graph
  slam.add_factor(prior);
  
  
  for (size_t i = 1; i < graph.getNbNodes(); i++) {
    isam::Pose2d_Node* new_pose_node = new isam::Pose2d_Node();
    slam.add_node(new_pose_node);
    pose_nodes.push_back(new_pose_node);
  }

  ROS_INFO_STREAM("Offline Map buliding - adding odometry/fused estimates");
  
  for (size_t i = 0; i < graph.getNbLinks(); i++) {
    const NDTFeatureLinkInterface &link = graph.getLinkInterface(i);
    if (link.getScore() < 0.) {
      Eigen::Affine3d Tincr_odom = link.getRelPose();
      isam::Pose2d odometry = convertEigenAffine3dToIsamPose2d(Tincr_odom);
      isam::Pose2d_Pose2d_Factor* constraint = new isam::Pose2d_Pose2d_Factor(pose_nodes[link.getRefIdx()], pose_nodes[link.getMovIdx()], odometry, noise3);
      slam.add_factor(constraint);
    }
  }

  ROS_INFO_STREAM("Offline Map buliding - adding DA estimates");

  for (size_t i = 0; i < graph.getNbLinks(); i++) {
    const NDTFeatureLinkInterface &link = graph.getLinkInterface(i);
    Eigen::Affine3d Tincr_odom = link.getRelPose();
    isam::Pose2d meassure = convertEigenAffine3dToIsamPose2d(Tincr_odom);
    isam::Pose2d_Pose2d_Factor* constraint = new isam::Pose2d_Pose2d_Factor(pose_nodes[link.getRefIdx()], pose_nodes[link.getMovIdx()], meassure, noise3);
    slam.add_factor(constraint);
    
  }

  ROS_INFO_STREAM("Offline Map buliding - optimizing...");
  
  slam.batch_optimization();
  
  ROS_INFO_STREAM("Offline Map buliding - updating pose est");
  
  for (size_t i = 0; i < graph.getNbNodes(); i++) {
    std::cout << pose_nodes[i]->value() << std::endl;
    graph.getNodeInterface(i).setPose(convertIsamPose2dToEigenAffine3d(pose_nodes[i]->value()));
  }
  
   
}


//! Quick check how the map looks using GT information to do DA.
inline void mapBuilderISAMOffline(std::vector<ndt_feature::NDTFeatureFrame> &frames, const Eigen::Affine3d &sensorPose, std::vector<std::pair<size_t, size_t> > &matches) {
    ROS_INFO_STREAM("Offline Map buliding - start, # frames:" << frames.size());
    
    isam::Slam slam; 

    // locally remember poses
    std::vector<isam::Pose2d_Node*> pose_nodes;
    
    isam::Noise noise3 = isam::Information(100. * isam::eye(3));
    isam::Noise noise2 = isam::Information(100. * isam::eye(2));
    
    // Add the nodes...
    
    // create a first pose (a node)
    isam::Pose2d_Node* new_pose_node = new isam::Pose2d_Node();
    // add it to the graph
    slam.add_node(new_pose_node);
    // also remember it locally
    pose_nodes.push_back(new_pose_node);
    
    // create a prior measurement (a factor)
    isam::Pose2d origin(0., 0., 0.);
    isam::Pose2d_Factor* prior = new isam::Pose2d_Factor(pose_nodes[0], origin, noise3);
    // add it to the graph
    slam.add_factor(prior);

    ROS_INFO_STREAM("Offline Map buliding - adding odometry");

    for (size_t i = 1; i < frames.size(); i++) {
      isam::Pose2d_Node* new_pose_node = new isam::Pose2d_Node();
      slam.add_node(new_pose_node);
      pose_nodes.push_back(new_pose_node);

      // connect to previous with odometry measurement
      // Get the odometry between frame i-1 to i.
      Eigen::Affine3d Tincr_odom = frames[i-1].odom.inverse()*frames[i].odom;
      isam::Pose2d odometry = convertEigenAffine3dToIsamPose2d(Tincr_odom);
      isam::Pose2d_Pose2d_Factor* constraint = new isam::Pose2d_Pose2d_Factor(pose_nodes[i-1], pose_nodes[i], odometry, noise3);
      slam.add_factor(constraint);
    }

    ROS_INFO_STREAM("Offline Map buliding - performing feature matching");

    // Do the feature matching - incremental
    for (size_t i = 1; i < frames.size(); i++) {
      std::vector<size_t> indices = getNDTFeatureFrameIndicesDistance(frames[i].gt, frames, i, 4., 100.);
      for (size_t j = 0; j < indices.size(); j++) {
	Eigen::Affine3d Tfeat_sensor_frame; 
	double score = ndtFeatureFrameMatchingFLIRT(frames[i], frames[indices[j]], Tfeat_sensor_frame);
	// Verify that the match is similar to the GT.
	Eigen::Affine3d Tfeat = sensorPose * Tfeat_sensor_frame * sensorPose.inverse();

	double dist, angular_dist;
        ndt_feature::distanceBetweenAffine3d(frames[i].gt*Tfeat, frames[indices[j]].gt, dist, angular_dist);
	// This is very much cheating here for now...
	if (dist < 0.1 && angular_dist < 0.1) {
	  // Good, add this as a factor.
	  isam::Pose2d Tfeat_ = convertEigenAffine3dToIsamPose2d(Tfeat);
	  isam::Pose2d_Pose2d_Factor* constraint = new isam::Pose2d_Pose2d_Factor(pose_nodes[i], pose_nodes[indices[j]], Tfeat_, noise3);
	  slam.add_factor(constraint);
	  matches.push_back(std::pair<int,int>(i, indices[j]));
	  ROS_INFO_STREAM("Adding[" << i << "," << indices[j] << "]");
	}
      }
    }
        
    ROS_INFO_STREAM("Offline Map buliding - optimizing...");

    slam.batch_optimization();

    ROS_INFO_STREAM("Offline Map buliding - updating pose est");
    
    for (size_t i = 0; i < frames.size(); i++) {
      std::cout << pose_nodes[i]->value() << std::endl;
      frames[i].pose = convertIsamPose2dToEigenAffine3d(pose_nodes[i]->value());
    }
    
  }

inline void mapBuilderISAMTest(std::vector<ndt_feature::NDTFeatureFrame> &frames) {
    //1 Add the odometry as contraints for each link
    //2a - First test using the current scan and match it with the N previous ones.
    //2b Use the gt information to find suitable matches
    
    isam::Slam slam; 

    // locally remember poses
    std::vector<isam::Pose2d_Node*> pose_nodes;
    
    isam::Noise noise3 = isam::Information(100. * isam::eye(3));
    isam::Noise noise2 = isam::Information(100. * isam::eye(2));
    
    // create a first pose (a node)
    isam::Pose2d_Node* new_pose_node = new isam::Pose2d_Node();
    // add it to the graph
    slam.add_node(new_pose_node);
    // also remember it locally
    pose_nodes.push_back(new_pose_node);
    
    // create a prior measurement (a factor)
    isam::Pose2d origin(0., 0., 0.);
    isam::Pose2d_Factor* prior = new isam::Pose2d_Factor(pose_nodes[0], origin, noise3);
    // add it to the graph
    slam.add_factor(prior);
    
    for (int i=1; i<4; i++) {
      // next pose
      isam::Pose2d_Node* new_pose_node = new isam::Pose2d_Node();
      slam.add_node(new_pose_node);
      pose_nodes.push_back(new_pose_node);
      
      // connect to previous with odometry measurement
      isam::Pose2d odometry(1., 0., 0.); // x,y,theta
      isam::Pose2d_Pose2d_Factor* constraint = new isam::Pose2d_Pose2d_Factor(pose_nodes[i-1], pose_nodes[i], odometry, noise3);
      slam.add_factor(constraint);
    }

    // create a landmark
    isam::Point2d_Node* new_point_node = new isam::Point2d_Node();
    slam.add_node(new_point_node);
    
    // create a pose and the landmark by a measurement
    isam::Point2d measure(5., 3.); // x,y
    isam::Pose2d_Point2d_Factor* measurement =
      new isam::Pose2d_Point2d_Factor(pose_nodes[1], new_point_node, measure, noise2);
    slam.add_factor(measurement);
    
    // optimize the graph
    slam.batch_optimization();
    
    // accessing the current estimate of a specific pose
    std::cout << "Pose 2: " << pose_nodes[2]->value() << std::endl;
    
    // printing the complete graph
    std::cout << std::endl << "Full graph:" << std::endl;
    slam.write(std::cout);
  }

} // namespace
