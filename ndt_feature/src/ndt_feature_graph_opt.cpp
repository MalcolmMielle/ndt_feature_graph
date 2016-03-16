#include <iostream>
#include <boost/program_options.hpp>
#include <boost/math/distributions/normal.hpp>
#include <boost/random.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#include <ndt_registration/ndt_matcher_d2d.h>
#include <ndt_registration/ndt_matcher_d2d_feature.h>
#include <ndt_feature/ndt_matcher_d2d_fusion.h>
#include <ndt_feature/ndt_feature_graph.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/cell_vector.h>
#include <ndt_map/lazy_grid.h>
#include <ndt_map/pointcloud_utils.h>

#include <ndt_feature/ndt_rviz.h>
#include <ros/ros.h>

#include <ndt_feature/utils.h>

using namespace std;
using namespace lslgeneric;
namespace po = boost::program_options;



int main(int argc, char** argv)
{
  ros::init(argc, argv, "ndt_feature_graph_opt");
  
  po::options_description desc("Allowed options");
  
  string file_name;
  int nb_nodes;
  desc.add_options()
    ("help", "produce help message")
    ("file_name", po::value<std::string>(&file_name)->default_value(std::string("")), "input graph file")
    ("nb_nodes", po::value<int>(&nb_nodes)->default_value(-1), "number of nodes -1 -> load all nodes")
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  
  if (vm.count("help"))
  {
    cout << desc << "\n";
    return 1;
  }
  
  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);


  ndt_feature::NDTFeatureGraph graph;
  
  std::cout << "Loading : " << file_name << std::endl;
  
  graph.load(file_name, nb_nodes);

  boost::shared_ptr<HistogramDistance<double> > histogram_dist(new SymmetricChi2Distance<double>());
  graph.setFeatureDistFunction(histogram_dist.get());


  std::cout << " Fuser parameters : " << graph.fuser_params_;
  std::cout << " Motion model parameters : " << graph.motion_params_;
  std::cout << " Sensor pose : ";
  lslgeneric::printTransf2d(graph.sensor_pose_);

  // Incremental pose estimate
  // Need the rel pose from odometry. Get the odometry covariance between two nodes.
  // Need the rel pose from fusion estimate - should contain covariance.
  
  // Non incremental pose estimates
  // Need the rel pose from ndt registration from muliple fused maps.
  // DA addressed using flirt, use the covariance from pose estimates to evaluate which matches that are consistent + check the robust matching openslam...


  // Any map data?
  std::cout << "getLastMap string : " << graph.getLastFeatureFuser()->map->getMyIndexStr() << std::endl;
  std::cout << "nbcells : " <<graph.getLastFeatureFuser()->map->getAllCells().size() << std::endl;
  std::cout << "nb of features : " << graph.getLastFeatureFuser()->featuremap.map.size() << std::endl;

  // Compute all possible matches...
  std::vector<ndt_feature::NDTFeatureNodeLink> matches = graph.computeAllPossibleLinks();
  for (size_t i = 0; i < matches.size(); i++) {
    std::cout << "matches[" << i << "] : " << matches[i] << std::endl;
  }
  
  for (size_t i = 0; i < matches.size(); i++) {
    if (matches[i].score < 0.1)
      std::cout << "good match : [" << matches[i].ref_idx << ", " << matches[i].mov_idx << "] : " << matches[i].score << std::endl;
  }

 
  // Draw the last map...
  
  ros::Rate loop_rate(1);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    marker_pub.publish(ndt_visualisation::markerNDTCells(*graph.getLastFeatureFuser()->map, 2, "lastmap"));
    
    marker_pub.publish(ndt_feature::interestPointMarkersFrameId(graph.getLastFeatureFuser()->featuremap.map, 0, std::string("/world")));
  }

  std::cout << " Done." << std::endl;
  return 1;
}
