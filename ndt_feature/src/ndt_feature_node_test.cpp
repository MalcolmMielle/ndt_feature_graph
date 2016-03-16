#include <iostream>
#include <boost/program_options.hpp>
#include <boost/math/distributions/normal.hpp>
#include <boost/random.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#include <ndt_feature/ndt_feature_rviz.h>
#include <ndt_registration/ndt_matcher_d2d.h>
#include <ndt_registration/ndt_matcher_d2d_feature.h>
#include <ndt_feature/ndt_matcher_d2d_fusion.h>
#include <ndt_feature/ndt_feature_graph.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/cell_vector.h>
#include <ndt_map/lazy_grid.h>
#include <ndt_map/pointcloud_utils.h>

#include <ros/ros.h>

#include <ndt_feature/utils.h>

using namespace std;
using namespace lslgeneric;
namespace po = boost::program_options;


void publishMarkerNDTFeatureNode(const ndt_feature::NDTFeatureNode &node, int id, const std::string &frame_id, ros::Publisher &marker_pub) {
  
  marker_pub.publish(ndt_feature::interestPointMarkersFrameId(node.map->featuremap.map, id, std::string("/world")));
  id++;
  marker_pub.publish(ndt_visualisation::markerNDTCells(*(node).map->map, id, frame_id));
}

// void publishMarkerNDTFeatureNode(const ndt_feature::NDTFeatureNode &node, int id, const std::string &frame_id, Eigen::Affine3d T, ros::Publisher &marker_pub) {
  
  
//   marker_pub.publish(ndt_feature::interestPointMarkersFrameId(node.map->featuremap.map, T, id, std::string("/world")));
//   id++;
//   std::vector<lslgeneric::NDTCell<pcl::PointXYZ>*> T_map = node.map->map->pseudoTransformNDT(T);
//   marker_pub.publish(ndt_visualisation::markerNDTCells(T_map, id, frame_id));
  
// }





int main(int argc, char** argv)
{
  ros::init(argc, argv, "ndt_feature_graph_opt");
  
  po::options_description desc("Allowed options");
  
  string file_name;
  int ref_idx, mov_idx;
  desc.add_options()
    ("help", "produce help message")
    ("file_name", po::value<std::string>(&file_name)->default_value(std::string("")), "input graph file")
    ("ref_index", po::value<int>(&ref_idx)->default_value(0), "reference node index value")
    ("mov_index", po::value<int>(&mov_idx)->default_value(1), "moving node index value")
    ("skip_ndt", "skip ndt match step")
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  
  if (vm.count("help"))
  {
    cout << desc << "\n";
    return 1;
  }
  
  bool skip_ndt = vm.count("skip_ndt");

  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);


  ndt_feature::NDTFeatureGraph graph;
  
  std::cout << "Loading : " << file_name << std::endl;
  
  graph.load(file_name);

  // Distance function pointer needed...
  boost::shared_ptr<HistogramDistance<double> > histogram_dist(new SymmetricChi2Distance<double>());
  graph.setFeatureDistFunction(histogram_dist.get());
  
  std::cout << " Fuser parameters : " << graph.fuser_params_;
  std::cout << " Motion model parameters : " << graph.motion_params_;
  std::cout << " Sensor pose : ";
  lslgeneric::printTransf2d(graph.sensor_pose_);

  ndt_feature::NDTFeatureNode &ref_node = graph.getNode(ref_idx);
  ndt_feature::NDTFeatureNode &mov_node = graph.getNode(mov_idx);
  
  // Do a feature based match -> T.
  // Compute the overlap
  // Do a NDT registration with T.
  // Compute the overlap
  Eigen::Affine3d T;
  T.setIdentity();
  std::cout << "overlap score (identity) : " << ndt_feature::overlapNDTOccupancyScore(ref_node, mov_node, T) << std::endl;
  Correspondences matches;
  ndt_feature::matchFeatureMap(ref_node.getFeatureMap(), mov_node.getFeatureMap(), matches, T);
  std::cout << "T (feat) : "; 
  lslgeneric::printTransf2d(T);
  std::cout << "overlap score (feat) : " << ndt_feature::overlapNDTOccupancyScore(ref_node, mov_node, T) << std::endl;

  if (skip_ndt) {
    lslgeneric::NDTMatcherD2D matcher_d2d;
    matcher_d2d.n_neighbours = 0;
    
    matcher_d2d.match(ref_node.getNDTMap(), mov_node.getNDTMap(), T,true);
    std::cout << "T (feat, then ndt) : "; 
    lslgeneric::printTransf2d(T);
    std::cout << "overlap score (feat) : " << ndt_feature::overlapNDTOccupancyScore(ref_node, mov_node, T) << std::endl;
  }

  ros::Rate loop_rate(1);

  std::vector<lslgeneric::NDTCell*> ndt_mov = mov_node.map->map->pseudoTransformNDT(T);
  lslgeneric::NDTMap* ndt_mov2 = mov_node.map->map->pseudoTransformNDTMap(T);


  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    publishMarkerNDTFeatureNode(ref_node, 0, "ref", marker_pub);

    marker_pub.publish(ndt_feature::interestPointMarkersFrameId(mov_node.map->featuremap.map, T, 2, std::string("/world")));
    marker_pub.publish(ndt_visualisation::markerNDTCells(ndt_mov));
    marker_pub.publish(ndt_visualisation::markerNDTCells(*ndt_mov2, 2, "mov"));
    
    // marker_pub.publish(ndt_visualisation::markerNDTCells(*(ref_node).map->map, 1, "ref"));
    
    // marker_pub.publish(ndt_feature::interestPointMarkersFrameId(ref_node.map->featuremap.map, 0, std::string("/world")));
    
    // marker_pub.publish(ndt_visualisation::markerNDTCells(*(mov_node).map->map, 2, "mov"));
    
    // marker_pub.publish(ndt_feature::interestPointMarkersFrameId(mov_node.map->featuremap.map, 1, std::string("/world")));
  }

  std::cout << " Done." << std::endl;
  return 1;
}
