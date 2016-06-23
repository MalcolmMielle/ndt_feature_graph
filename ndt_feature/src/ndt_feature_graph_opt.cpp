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
#include <ndt_feature/ndt_offline_mapper.h>
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
  int nb_nodes, min_idx_dist, nb_neighbours;
  double max_score, max_dist, max_angular_dist;
  desc.add_options()
    ("help", "produce help message")
    ("file_name", po::value<std::string>(&file_name)->default_value(std::string("")), "input graph file")
    ("nb_nodes", po::value<int>(&nb_nodes)->default_value(-1), "number of nodes -1 -> load all nodes")
    ("compute_links", "(re)compute links")
    ("no_optimize", "don't perform any optimizations")
    ("only_odometry", "only used odometry links")
    ("use_incrT", "use the global T as base for incremental odometry links")
    ("use_fuseT", "use the fused estimates as base for incremental odometry links")
    ("draw_no_links", "draw links using rviz markers")
    ("force2d", "force all transformations to be using x,y,theta only")
    ("max_score", po::value<double>(&max_score)->default_value(0.1), "max allowed matching score (lower is better) for DA")
    ("max_dist", po::value<double>(&max_dist)->default_value(1.0), "max allowed distance for matching for DA")
    ("max_angular_dist", po::value<double>(&max_angular_dist)->default_value(0.2), "max allowed rotational error for matching")
    ("min_idx_dist", po::value<int>(&min_idx_dist)->default_value(2), "min distance in indexes between nodes, to avoid that consectuive scans are matched")
    ("skip_ndt", "if ndt should not be used for improving the T obtained from DA")
    ("nb_neighbours", po::value<int>(&nb_neighbours)->default_value(0), "number of neighbours used in the ndt matching search")
    ("keep_score", "if the old score should be keep while using the ndt registration between DA.")
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  
  if (vm.count("help"))
  {
    cout << desc << "\n";
    return 1;
  }
  
  bool compute_links = vm.count("compute_links");
  bool optimize = !vm.count("no_optimize");
  bool only_odometry = vm.count("only_odometry");
  bool use_incrT = vm.count("use_incrT");
  bool use_fuseT = vm.count("use_fuseT");
  bool force2d = vm.count("force2d");
  bool use_ndt = !vm.count("skip_ndt");
  bool keep_score = vm.count("keep_score");
  bool draw_links = !vm.count("draw_no_links");

  std::cout << "min_idx_dist     : " << min_idx_dist << std::endl;
  std::cout << "max_dist         : " << max_dist << std::endl;
  std::cout << "max_angular_dist : " << max_angular_dist << std::endl;
  std::cout << "max_score        : " << max_score << std::endl;

  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud2>("pc", 10);

  ndt_feature::NDTFeatureGraph graph;
  
  std::cout << "Loading : " << file_name << std::endl;
  
  graph.load(file_name, nb_nodes);

  std::cout << "Number of links : " << graph.getNbLinks() << std::endl;

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

  std::vector<ndt_feature::NDTFeatureLink> incremental_links;
  if (use_incrT) {
    incremental_links = graph.getIncrementalLinks(); 
  }
  else if (use_fuseT) {
    incremental_links = graph.getAllIncrementalFuseLinks() ; 
  }
  else {
    incremental_links = graph.getAllIncrementalOdomLinks();
  }

  std::vector<ndt_feature::NDTFeatureLink> all_matched_links;

  if (graph.getNbLinks() == 0 || compute_links) {
    // Compute all possible matches...
    graph.clearAllLinks();
    all_matched_links = graph.computeAllPossibleLinks();
    graph.appendLinks(all_matched_links);
    graph.saveLinkFile(".links");
  }
  else {
    all_matched_links = graph.getCurrentLinks(); // This are all links loaded from file
  }

  if (use_ndt) {
    graph.updateLinksUsingNDTRegistration(all_matched_links, nb_neighbours, keep_score);
  }

  if (force2d) {
    graph.force2D();
  }

  std::cout << "total amount of matches : " << all_matched_links.size() << std::endl;

  if (optimize) {
    bool done = false;
    int opt_counter = 0;
    size_t prev_nb_links = 0;
    while (!done) {
      std::cout << "opt counter : " << ++opt_counter << std::endl;
      graph.clearAllLinks();
      graph.appendLinks(incremental_links);
      std::vector<ndt_feature::NDTFeatureLink> links = graph.getValidLinks(all_matched_links, max_score, max_dist, max_angular_dist, min_idx_dist);
      std::cout << "links.size() : " << links.size() << std::endl;
      if (!links.empty() && !only_odometry) {
        graph.appendLinks(links);
        optimizeGraphUsingISAM(graph);
      }
      else {
        done = true;
      }
      if (prev_nb_links == links.size()) { // No more links added...
        done = true;
      }
      prev_nb_links = links.size();
    }
  }
  
  ros::Rate loop_rate(1);

  pcl::PointCloud<pcl::PointXYZ> pc;
  for (size_t i = 0; i < graph.getNbNodes(); i++) {
    pc += graph.getNode(i).getGlobalPointCloud();
  }
  
  sensor_msgs::PointCloud2 cloud_msg; 
  pcl::toROSMsg(pc, cloud_msg );
  cloud_msg.header.frame_id = std::string("/world");

  std::vector<Eigen::Affine3d> odom_global = graph.getNodePosesFromOdometry();
  
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    // marker_pub.publish(ndt_visualisation::markerNDTCells(*graph.getLastFeatureFuser()->map, 2, "lastmap"));
    
    // marker_pub.publish(ndt_feature::interestPointMarkersFrameId(graph.getLastFeatureFuser()->featuremap.map, 0, std::string("/world")));
    
    ndt_feature::publishMarkerNDTFeatureNodes(graph, marker_pub);
    if (draw_links) {
      ndt_feature::publishMarkerNDTFeatureLinks(graph, marker_pub);
    }
    pc_pub.publish(cloud_msg);

    for (size_t i = 0; i < odom_global.size(); i++) {
      marker_pub.publish(ndt_feature::poseArrowMarkerEigen(odom_global[i], i, 1, "/world"));
    }
    
  }

  std::cout << " Done." << std::endl;
  return 1;
}
