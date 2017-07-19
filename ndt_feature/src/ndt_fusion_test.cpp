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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ndt_fusion_test");
  
  po::options_description desc("Allowed options");
  
  string file_name;
  int ref_idx, mov_idx;
  int nb_neighbours;
  double Tx,Ty,Tth;
  double Cd,Ct,Dd,Dt,Td,Tt;
  desc.add_options()
    ("help", "produce help message")
    ("file_name", po::value<std::string>(&file_name)->default_value(std::string("")), "input graph file")
    ("ref_index", po::value<int>(&ref_idx)->default_value(0), "reference node index value")
    //    ("mov_index", po::value<int>(&mov_idx)->default_value(1), "moving node index value")
    ("nb_neighbours", po::value<int>(&nb_neighbours)->default_value(0), "number of neighbours used in the ndt matching search")
    ("use_initial_guess", "use odometry as initial guess")
    ("use_ndt", "use ndt in the optimization")
    ("use_odom", "create a ndt cells based on odometry")
    ("use_feat", "use feature in the optimization")
    ("no_step_control", "do not use any step control in optimization")
    ("offset_Tx", po::value<double>(&Tx)->default_value(0.), "offset to T (initial pose estimate)")
    ("offset_Ty", po::value<double>(&Ty)->default_value(0.), "offset to T (initial pose estimate)")
    ("offset_Tth", po::value<double>(&Tth)->default_value(0.), "offset to T (initial pose estimate)")
    ("no_est", "if no estimate should be computed")
    ("no_markers", "if no visualization markers should be published")
    ("add_odom_to_ndtmap", "if the odometry estimate based cells should be added directly to the ndt based maps")
    ("Cd", po::value<double>(&Cd)->default_value(0.005), "motion model sideways (dist)")
    ("Ct", po::value<double>(&Ct)->default_value(0.01), "motion model sideways (rot)")
    ("Dd", po::value<double>(&Dd)->default_value(0.001), "motion model front (dist)")
    ("Dt", po::value<double>(&Dt)->default_value(0.01), "motion model front (rot)")
    ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  
  if (vm.count("help"))
  {
    cout << desc << "\n";
    return 1;
  }

  mov_idx = ref_idx + 1;
  
  bool use_initial_guess = vm.count("use_initial_guess");
  bool use_ndt = vm.count("use_ndt");
  bool use_odom = vm.count("use_odom");
  bool use_feat = vm.count("use_feat");
  bool step_control = !vm.count("no_step_control");
  bool compute_est = !vm.count("no_est");
  bool add_odom_to_ndtmap = vm.count("add_odom_to_ndtmap");
  bool no_markers = vm.count("no_markers");

  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Publisher pcref_pub = nh.advertise<sensor_msgs::PointCloud2>("pc_ref", 10);
  ros::Publisher pcmov_pub = nh.advertise<sensor_msgs::PointCloud2>("pc_mov", 10);
  
  Eigen::Affine2d offset = lslgeneric::getAffine2d(Tx, Ty, Tth);
  Eigen::Affine3d offset3d = lslgeneric::eigenAffine2dTo3d(offset);

  ndt_feature::NDTFeatureGraph graph;
  
  std::cout << "Loading : " << file_name << std::endl;
  
  graph.load(file_name, mov_idx+1);

  // Distance function pointer needed...
  boost::shared_ptr<HistogramDistance<double> > histogram_dist(new SymmetricChi2Distance<double>());
  graph.setFeatureDistFunction(histogram_dist.get());
  graph.motion_params_.Cd = Cd;
  graph.motion_params_.Ct = Ct;
  graph.motion_params_.Dd = Dd;
  graph.motion_params_.Dt = Dt;

  std::cout << " Fuser parameters : " << graph.fuser_params_;
  std::cout << " Motion model parameters : " << graph.motion_params_;
  std::cout << " Sensor pose : ";
  lslgeneric::printTransf2d(graph.sensor_pose_);

  std::cout << "graph.getNbNodes() : " << graph.getNbNodes() << std::endl;
  std::cout << "ref_idx : " << ref_idx << std::endl;
  std::cout << "mov_idx : " << mov_idx << std::endl;
  

  ndt_feature::NDTFeatureNode &ref_node = graph.getNode(ref_idx);
  ndt_feature::NDTFeatureNode &mov_node = graph.getNode(mov_idx);
  Eigen::Affine3d T, Tfeat, Tmotion;
  Eigen::MatrixXd T_cov(6,6);
  T_cov.setIdentity();

  Tmotion = ref_node.Tlocal_odom;
  std::cout << "Odometry used : " << std::endl;
  lslgeneric::printTransf2d(Tmotion);
  
  // Odometry 'constraints'
  ndt_feature::MotionModel2d motion(graph.motion_params_);
  ndt_feature::Pose2d relpose(Tmotion.translation()[0],
                                 Tmotion.translation()[1],
                                 Tmotion.rotation().eulerAngles(0,1,2)[2]);
  ndt_feature::Pose2dCov relposecov = motion.getPose2dCov(relpose);
  Eigen::Matrix3d odom_cov = relposecov.cov;
  odom_cov(2,2) = 0.01; // This is the height in the ndt feature vec and not rotational variance.
  lslgeneric::NDTCell ndt_odom_cell;
  ndt_odom_cell.setMean(Eigen::Vector3d(0.,0.,0.));
  ndt_odom_cell.setCov(odom_cov);
  lslgeneric::NDTCell ndt_odom_cell_prev;
  ndt_odom_cell_prev.setMean(Tmotion.translation());
  ndt_odom_cell_prev.setCov(odom_cov);
  std::cout << " Pose + Cov from motion : " << relposecov << std::endl;
  std::cout << "..." << std::endl;

  lslgeneric::CellVector cv_feature_ref;
  lslgeneric::CellVector cv_feature_mov;

  Correspondences feature_matches;
  std::vector<std::pair<int, int> > corr;

  std::cout << "maching features" << std::endl;
  std::cout << "ref_node.getFeatureMap().getMap().size() : " << ref_node.getFeatureMap().getMap().size() << std::endl;
  std::cout << "mov_node.getFeatureMap().getMap().size() : " << mov_node.getFeatureMap().getMap().size() << std::endl;

  double match_score_feature = matchFeatureMap(ref_node.getFeatureMap(), mov_node.getFeatureMap(), feature_matches, Tfeat);
  std::cout << "... score : " << match_score_feature << std::endl;
  if (use_feat)
  {
    std::cout << "adding features to ndt cells" << std::endl;
    Eigen::Matrix3d cov;
    cov << 0.002, 0., 0., 0., 0.002, 0., 0., 0., 0.001; // Feature covarance...

    ndt_feature::convertCorrespondencesToCellvectorsFixedCovWithCorr(feature_matches, cov, &cv_feature_ref, &cv_feature_mov, corr);
  }

  lslgeneric::NDTMap ndt_feature_ref(&cv_feature_ref);
  lslgeneric::NDTMap ndt_feature_mov(&cv_feature_mov);


  if (use_odom)
  {
    std::cout << "adding odometry" << std::endl;
    addNDTCellToMap(&ndt_feature_ref, &ndt_odom_cell_prev);
    addNDTCellToMap(&ndt_feature_mov, &ndt_odom_cell);
    int tmp_size = corr.size(); 
    corr.push_back(std::pair<int,int>(tmp_size, tmp_size));
  }

  if (add_odom_to_ndtmap) {
    setNDTCellToMap(&ref_node.getNDTMap(), &ndt_odom_cell_prev);
    setNDTCellToMap(&mov_node.getNDTMap(), &ndt_odom_cell);
  }
    
  T = Tmotion*offset3d;
  bool valid_match;
  if (compute_est) {
    valid_match = matchFusion( ref_node.getNDTMap(),
                                  mov_node.getNDTMap(),
                                  ndt_feature_ref, 
                                  ndt_feature_mov,
                                  corr,
                                  T,
                                  T_cov,
                                  use_initial_guess,
                                  use_ndt,
                                  use_feat || use_odom,
                                  step_control);
  }
  std::cout << "estimated T : " << std::endl;
  lslgeneric::printTransf2d(T);

  ros::Rate loop_rate(1);

  std::vector<lslgeneric::NDTCell*> ndt_mov = mov_node.map->map->pseudoTransformNDT(T);
  lslgeneric::NDTMap* ndt_mov2 = mov_node.map->map->pseudoTransformNDTMap(T);

  lslgeneric::NDTMap* ndt_feat_mov = ndt_feature_mov.pseudoTransformNDTMap(T);


  sensor_msgs::PointCloud2 cloud_msg_ref; 
  pcl::toROSMsg(ref_node.getLocalPointCloud(), cloud_msg_ref );
  cloud_msg_ref.header.frame_id = std::string("/world");
  
  sensor_msgs::PointCloud2 cloud_msg_mov; 
  pcl::PointCloud<pcl::PointXYZ> pc_mov = mov_node.getLocalPointCloud();
  lslgeneric::transformPointCloudInPlace(T, pc_mov);
  pcl::toROSMsg(pc_mov, cloud_msg_mov );
  cloud_msg_mov.header.frame_id = std::string("/world");
  
  

  while (ros::ok() && !no_markers) {
    ros::spinOnce();
    loop_rate.sleep();
    publishMarkerNDTFeatureNode(ref_node, 0, "ref", marker_pub);
    publishMarkerNDTFeatureNode(mov_node, 1, "mov", marker_pub);

    marker_pub.publish(ndt_feature::interestPointMarkersFrameId(mov_node.map->featuremap.map, T, 2, std::string("/world")));
    marker_pub.publish(ndt_visualisation::markerNDTCells(ndt_mov));
    marker_pub.publish(ndt_visualisation::markerNDTCells(*ndt_mov2, 2, "mov"));
    marker_pub.publish(ndt_visualisation::markerNDTCells(ndt_feature_ref, 1, "feat_ref"));
    marker_pub.publish(ndt_visualisation::markerNDTCells(*ndt_feat_mov, 2, "feat_mov"));
    marker_pub.publish(ndt_visualisation::markerNDTCells(ndt_feature_mov, 0, "feat_mov2"));

    marker_pub.publish(ndt_feature::correspondenceMarkers(feature_matches, std::string("/world")));

    pcref_pub.publish(cloud_msg_ref);
    pcmov_pub.publish(cloud_msg_mov);
  }

  delete ndt_mov2;
  delete ndt_feat_mov;
  for (size_t i = 0; i < ndt_mov.size(); i++) {
     delete ndt_mov[i];
  }

  std::cout << " Done." << std::endl;
  return 1;
}
