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

#include <ndt_feature/utils.h>

using namespace std;
using namespace lslgeneric;
namespace po = boost::program_options;

int main(int argc, char** argv)
{
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

  Eigen::Affine2d offset = lslgeneric::getAffine2d(Tx, Ty, Tth);
  Eigen::Affine3d offset3d = lslgeneric::eigenAffine2dTo3d(offset);

  ndt_feature::NDTFeatureGraph graph;
  
  std::cout << "Loading : " << file_name << std::endl;
  
  graph.load(file_name, mov_idx+1);

  // Distance function pointer needed...
  boost::shared_ptr<HistogramDistance<double> > histogram_dist(new SymmetricChi2Distance<double>());
  graph.setFeatureDistFunction(histogram_dist.get());

  lslgeneric::NDTMap* ndt_tmp = graph.getMap()->pseudoTransformNDTMap(offset3d);
  lslgeneric::NDTMap* ndt_tmp2 = ndt_tmp->pseudoTransformNDTMap(offset3d);

  lslgeneric::NDTCell ndt_odom_cell;
  ndt_odom_cell.setMean(Eigen::Vector3d(0.,0.,0.));
  //  ndt_odom_cell.setCov(odom_cov);
  addNDTCellToMap(ndt_tmp2, &ndt_odom_cell);



  delete ndt_tmp;
  std::cout << "...." << std::endl;
  delete ndt_tmp2;

  std::cout << ".... done ....." << std::endl;

  return 1;
}
