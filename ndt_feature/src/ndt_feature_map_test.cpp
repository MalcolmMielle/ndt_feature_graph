#include <iostream>
#include <boost/program_options.hpp>
#include <boost/math/distributions/normal.hpp>
#include <boost/random.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#include <ndt_feature/ndt_feature_map.h>
#include <ndt_feature/ndt_feature_rviz.h>
#include <ros/ros.h>

#include <ndt_feature/utils.h>

using namespace std;
using namespace lslgeneric;
namespace po = boost::program_options;



int main(int argc, char** argv)
{
  ros::init(argc, argv, "ndt_feature_map_test");
  
  po::options_description desc("Allowed options");
  
  string file_name_ref;
  string file_name_mov;
  desc.add_options()
    ("help", "produce help message")
    ("file_name_ref", po::value<std::string>(&file_name_ref)->required(), "input feature file - reference")
    ("file_name_mov", po::value<std::string>(&file_name_mov)->required(), "input feature file - to be aligned")
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


  ndt_feature::NDTFeatureMap map_ref, map_mov;
  
  std::cout << "Loading - ref : " << file_name_ref << std::endl;
  std::cout << "Loading - mov : " << file_name_mov << std::endl;
  
  // Distance function pointer needed...
  boost::shared_ptr<HistogramDistance<double> > histogram_dist(new SymmetricChi2Distance<double>());

  map_ref.load(file_name_ref);
  map_mov.load(file_name_mov);

  map_ref.setDistFunction(histogram_dist.get());
  map_mov.setDistFunction(histogram_dist.get());
  std::cout << "nb of features - ref : " << map_ref.map.size() << std::endl;
  std::cout << "nb of features - mov : " << map_mov.map.size() << std::endl;

  // Check the descriptors (!)


  Eigen::Affine3d T;

  Correspondences matches;
  boost::shared_ptr<RansacFeatureSetMatcher> ransac_(new RansacFeatureSetMatcher(0.0599, 0.9, 0.1, 0.6, 0.0499, false));

  //  RansacFeatureSetMatcher ransac(0.0599, 0.9, 0.1, 0.6, 0.0499, false);
  OrientedPoint2D transform;

  const double score = ransac_->matchSets(map_ref.map, map_mov.map, transform, matches);
  
  std::cerr << "ransac score : " << score << std::endl;
  ndt_feature::convertOrientedPoint2DToEigen(transform, T);

  //  matchFeatureMap(map_ref, map_mov, matches, transform);
  std::cout << "nb correspondences : " << matches.size() << std::endl;

  geometry_msgs::Pose pose;
  tf::poseEigenToMsg(T, pose);

  visualization_msgs::Marker m_ref = ndt_feature::interestPointMarkersFrameId(map_ref.map, 0, "/world");
  visualization_msgs::Marker m_mov = ndt_feature::interestPointMarkersFrameId(map_mov.map, 1, "/world");
  visualization_msgs::Marker m_mov2 = ndt_feature::interestPointMarkersFrameId(map_mov.map, pose, 2, "/world");

  visualization_msgs::Marker m_matches = ndt_feature::correspondenceMarkers(matches, "/world");
   ros::Rate loop_rate(1);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    marker_pub.publish(m_ref);
    marker_pub.publish(m_mov);
    marker_pub.publish(m_mov2);
    marker_pub.publish(m_matches);
    std::cout << ".";
 }

  std::cout << " Done." << std::endl;
  return 1;
}
