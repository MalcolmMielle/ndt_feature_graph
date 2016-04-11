#include <iostream>
#include <boost/program_options.hpp>
#include <ndt_feature/utils.h>

using namespace std;
using namespace lslgeneric;
namespace po = boost::program_options;

int main(int argc, char** argv)
{
  po::options_description desc("Allowed options");
  
  string file_name;
  int nb_nodes;
  desc.add_options()
    ("help", "produce help message")
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  
  if (vm.count("help"))
  {
    cout << desc << "\n";
    return 1;
  }

  {
    double offset = 0.001;
    
    Eigen::Affine3d a3d = Eigen::Translation3d(2., 0.1, 0.) *
      Eigen::AngleAxisd(-M_PI+offset, Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(M_PI+offset, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI+0.9, Eigen::Vector3d::UnitZ());
    
    Eigen::Affine3d a3d_force2d = forceEigenAffine3dTo2d(a3d);
    
    std::cout << "a3d : " << std::endl;
    printTransf(a3d);
    
    std::cout << "a3d_forced2d : " << std::endl;
    printTransf(a3d_force2d);
  }

  std::cout << "-----------------" << std::endl;

  {
    double offset = 0.001;
    
    Eigen::Affine3d a3d = Eigen::Translation3d(2., 0.1, 0.) *
      Eigen::AngleAxisd(0.00104309, Eigen::Vector3d::UnitX())*Eigen::AngleAxisd( -0.000766065, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-3.11517, Eigen::Vector3d::UnitZ());
    
    Eigen::Affine3d a3d_force2d = forceEigenAffine3dTo2d(a3d);
    
    std::cout << "a3d : " << std::endl;
    printTransf(a3d);
    
    std::cout << "a3d_forced2d : " << std::endl;
    printTransf(a3d_force2d);
  }

}
