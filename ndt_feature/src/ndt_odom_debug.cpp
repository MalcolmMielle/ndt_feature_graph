#include <iostream>
#include <boost/program_options.hpp>
#include <boost/math/distributions/normal.hpp>
#include <boost/random.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#include <ndt_registration/ndt_matcher_d2d.h>
#include <ndt_registration/ndt_matcher_d2d_feature.h>
#include <ndt_feature/ndt_matcher_d2d_fusion.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/cell_vector.h>
#include <ndt_map/lazy_grid.h>
#include <ndt_map/pointcloud_utils.h>

#include <ndt_feature/ndt_rviz.h>
#include <ndt_feature/utils.h>

using namespace std;
using namespace lslgeneric;
namespace po = boost::program_options;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ndt_feature_test");

    cout << "-----------------------------------------------------------------------" << endl;
    cout << "Small test program of ndt matcher with initial estimate (i.e. odometry)" << endl;
    cout << "-----------------------------------------------------------------------" << endl;


    po::options_description desc("Allowed options");
    Eigen::Matrix<double,6,1> pose_increment_v, odom_increment_v;
    string static_file_name, moving_file_name;
    int nb_clusters, nb_points, n_neighbours;
    double std_dev, std_dev_odom, min, max, resolution, max_range;
    desc.add_options()
    ("help", "produce help message")
    ("x", po::value<double>(&pose_increment_v(0))->default_value(0.), "x pos gt offset")
    ("y", po::value<double>(&pose_increment_v(1))->default_value(0.), "y pos gt offset")
    ("Z", po::value<double>(&pose_increment_v(5))->default_value(0.), "z axis rot gt offset")
    ("odom_x", po::value<double>(&odom_increment_v(0))->default_value(0.), "x pos odom offset")
    ("odom_y", po::value<double>(&odom_increment_v(1))->default_value(0.), "y pos odom offset")
    ("odom_Z", po::value<double>(&odom_increment_v(5))->default_value(0.), "z axis rot odom offset")

    ("nb_clusters", po::value<int>(&nb_clusters)->default_value(2), "number of clusters")
    ("nb_points", po::value<int>(&nb_points)->default_value(50), "number of points per clusters")
      ("std_dev", po::value<double>(&std_dev)->default_value(0.03), "standard deviation of the points drawn from a normal distribution (here independent on the axes")
      ("std_dev_odom", po::value<double>(&std_dev_odom)->default_value(0.03), "std devation of odometry (1D)")
      ("min", po::value<double>(&min)->default_value(-10), "minimum center point")
    ("max", po::value<double>(&max)->default_value(10), "maximum center point")
    ("resolution", po::value<double>(&resolution)->default_value(1), "resolution of cells")
      ("max_range", po::value<double>(&max_range)->default_value(30.), "max x value")
      ("n_neighbours", po::value<int>(&n_neighbours)->default_value(0), "number of neighbours cells (in addition to the closes one) used in matching")
      ("usegt", "if the ground truth should be used as initial estimate")
      ("show_p2p", "if the point to point matching should be visualized")
      ("show_d2d", "if the point to point matching should be visualized")
      ("nuse_NDT", "fusion matching debug param : if NDT should *NOT* be used")
      ("nuse_feat", "fusion matching debug param : if features should *NOT* be used")
      ("nstep_control", "fusion matching debug param : if step_control should *NOT* be used")
      ("discard_cells", "if cells where first and last sensor readings fall into should be discarded - use with nb_clusters = 1")
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
    bool usegt = vm.count("usegt");
    bool use_singleres = vm.count("singleres");
    bool use_irregular_grid = vm.count("irregular_grid");
    
    bool use_NDT = !vm.count("nuse_NDT");
    bool use_feat = !vm.count("nuse_feat");
    bool step_control = !vm.count("nstep_control");
    
    bool show_p2p = vm.count("show_p2p"); 
    bool show_d2d = vm.count("show_d2d");
    bool discard_cells = vm.count("discard_cells");
           
    pcl::PointCloud<pcl::PointXYZ> static_pc, moving_pc;
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> gt_transform, odom_transform;
    Eigen::MatrixXd T_cov(6,6);
    T_cov.setIdentity();

    // Generate the static point cloud - parallel lines as seen from a cooridor.
    boost::mt19937 rng;
    boost::uniform_real<> ud(min,max);
    boost::normal_distribution<> nd(0.0, std_dev);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > var_uni(rng, ud);
    
    for (int i = 0; i < nb_clusters; i++)
    {
        std::vector<size_t> indices;
        double c_x = 0.;
        double c_y = var_uni();
        if (i == 0)
          c_y = -1;
        if (i == 1)
          c_y = 1;
        double c_z = 0.;
        for (int j = 0; j < nb_points; j++)
        {
          double angle = M_PI*j/(2.*nb_points);
          double x = tan(angle)*fabs(c_y);
          if (fabs(x) > max_range)
            continue;
          static_pc.push_back(pcl::PointXYZ(c_x+var_nor()+x, c_y+var_nor(), c_z+var_nor())); // Simply to get some intial similarity betwen a range scanner... more readings closer than further away...
        }
    }
    ros::Rate loop_rate(1);

       while (ros::ok())
    {
      std::cout << "--1" << std::endl;
      pose_increment_v(0) += 0.04;
    gt_transform = Eigen::Translation<double,3>(pose_increment_v(0),pose_increment_v(1),pose_increment_v(2))*
                   Eigen::AngleAxis<double>(pose_increment_v(3),Eigen::Vector3d::UnitX()) *
                   Eigen::AngleAxis<double>(pose_increment_v(4),Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxis<double>(pose_increment_v(5),Eigen::Vector3d::UnitZ()) ;

      std::cout << "--2" << std::endl;
    moving_pc = lslgeneric::transformPointCloud<pcl::PointXYZ>(gt_transform, static_pc);

    odom_transform = Eigen::Translation<double,3>(odom_increment_v(0),odom_increment_v(1),odom_increment_v(2))*
      Eigen::AngleAxis<double>(odom_increment_v(3),Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxis<double>(odom_increment_v(4),Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxis<double>(odom_increment_v(5),Eigen::Vector3d::UnitZ()) ;
          std::cout << "--3" << std::endl;


    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T_p2p, T_d2d, T_fusion;
    T_p2p.setIdentity();
    T_d2d.setIdentity();
    T_fusion.setIdentity();
    
    T_p2p = odom_transform;
    T_d2d = odom_transform;
    T_fusion = odom_transform;

    if (usegt)
    {
        T_p2p = gt_transform;
        T_d2d = gt_transform;
        T_fusion = gt_transform;
    }

    std::cout << "d2d - INITIAL" << std::endl;
    printTransf(T_d2d);

    std::vector<double> resolutions;
    if (use_singleres)
        resolutions.push_back(1.);
    NDTMatcherD2D matcher(use_irregular_grid,!use_singleres,resolutions);
    matcher.n_neighbours = n_neighbours;
    matcher.match(static_pc, moving_pc,  T_p2p, true);
    
    SpatialIndex* index1_lazzy = NULL;
    SpatialIndex* index2_lazzy = NULL;
       
    index1_lazzy = new LazyGrid(resolution);
    index2_lazzy = new LazyGrid(resolution);

    Eigen::Vector3d map_size(80., 80., 2.);
    
    std::cout << "before creating static NDTMap" << std::endl;
    NDTMap ndt(index1_lazzy);
    ndt.setMapSize(map_size[0], map_size[1], map_size[2]);
    ndt.loadPointCloud (static_pc );
    ndt.computeNDTCells();
    Eigen::Vector3d ndt_centroid;
    ndt.getCentroid(ndt_centroid[0], ndt_centroid[1], ndt_centroid[2]);
    
    std::cout << "before creating moving NDTMap" << std::endl;
    NDTMap mov(index2_lazzy);
    mov.loadPointCloud( moving_pc );
    mov.computeNDTCells();
    Eigen::Vector3d mov_centroid;
    mov.getCentroid(mov_centroid[0], mov_centroid[1], mov_centroid[2]);
    std::cout << "mov_centroid : " << mov_centroid << std::endl;     // THIS IS THE PROBLEM(!), the centroid stays the same...

    mov.loadPointCloudCentroid( moving_pc, ndt_centroid, ndt_centroid, map_size, 60.);
    mov.computeNDTCells();

    if (discard_cells) {
      // Look at the first and last point in the range data. Simply invalidate these cells since their distribution is very likely to be wrongly estimated. This is important for all sensor data that are not omni-directional.
      std::cout << "----- discarding cells -------" << std::endl;
      NDTCell *cell_front, *cell_back;
      mov.getCellAtPoint(moving_pc.front(), cell_front);
      mov.getCellAtPoint(moving_pc.back(), cell_back);
      cell_front->hasGaussian_ = false;
      cell_back->hasGaussian_ = false;
    }


    std::cout << "------------- MATCHING d2d ------------------" << std::endl;
    matcher.match( ndt, mov, T_d2d, true );

    std::cout << "--------------- Fusion matching ----------------" << std::endl;
    
    // Add odometry reading as a NDT to NDT correspondance feature
    CellVector* index1_cellvec = NULL;
    CellVector* index2_cellvec = NULL;
    index1_cellvec = new CellVector();
    index2_cellvec = new CellVector();
    // Add odometry ndt cell...
    
    NDTCell* ndt_odom_cell = new NDTCell();
    ndt_odom_cell->setMean(Eigen::Vector3d(0., 0., 0.));
    Eigen::Matrix3d cov;
    cov << std_dev_odom*std_dev_odom, 0., 0., 0., 0.1, 0., 0., 0., 0.1;
    ndt_odom_cell->setCov(cov);
    index1_cellvec->addNDTCell(ndt_odom_cell);
    index2_cellvec->addNDTCell(ndt_odom_cell);
    std::vector<std::pair<int,int> > corresp;
    corresp.push_back(std::pair<int,int>(0,0));
    NDTMap ndt_feat(index1_cellvec);
    NDTMap mov_feat(index2_cellvec);
    ndt_feat.computeNDTCellsSimple();
    mov_feat.computeNDTCellsSimple();
    
    matchFusion(ndt, mov,
                ndt_feat, mov_feat,
                corresp,
                T_fusion,
                T_cov,
                true,
                use_NDT,
                use_feat,
                step_control,
                100,
                n_neighbours);
    



    std::cout << "GT" << std::endl;
    printTransf(gt_transform);
    std::cout << "odom" << std::endl;
    printTransf(odom_transform);
    std::cout << "d2d" << std::endl;
    printTransf(T_d2d);
    std::cout << "p2p" << std::endl;
    printTransf(T_p2p);
    std::cout << "fusion" << std::endl;
    printTransf(T_fusion);
    
 
    cout << "done." << endl;


    NDTMap *aligned_ndt = NULL;
    NDTMap *aligned_odom_ndt = NULL;
    if (show_p2p) {
      aligned_ndt = mov.pseudoTransformNDTMap(T_p2p);
    }
    else if (show_d2d) {
      aligned_ndt = mov.pseudoTransformNDTMap(T_d2d);
    }
    else {
      aligned_ndt = mov.pseudoTransformNDTMap(T_fusion);
      aligned_odom_ndt = mov_feat.pseudoTransformNDTMap(T_fusion);
    }

    std::cout << "-1" << std::endl;

    //ros::Rate loop_rate(1);

    //    while (ros::ok())
    //{
      marker_pub.publish(ndt_visualisation::markerNDTCells(ndt, 0));
      marker_pub.publish(ndt_visualisation::markerNDTCells(mov, 1));
      if (aligned_ndt != NULL)
        marker_pub.publish(ndt_visualisation::markerNDTCells(*aligned_ndt, 2));
    std::cout << "-2" << std::endl;


      marker_pub.publish(ndt_visualisation::markerNDTCells(ndt_feat, 3));
      if (aligned_odom_ndt != NULL) {
        marker_pub.publish(ndt_visualisation::markerNDTCells(*aligned_odom_ndt, 4));
        marker_pub.publish(ndt_visualisation::markerCellVectorCorrespondances(ndt_feat, *aligned_odom_ndt, corresp));
      }
    std::cout << "-3" << std::endl;

      ros::spinOnce();
      loop_rate.sleep();
      //}
    std::cout << "-4" << std::endl;

    delete index1_lazzy;
    std::cout << "-5" << std::endl;

    delete index2_lazzy;
    if (aligned_ndt != NULL)
      delete aligned_ndt;
    if (aligned_odom_ndt != NULL)
      delete aligned_odom_ndt;
    std::cout << "-6" << std::endl;

    delete index1_cellvec;
    std::cout << "-7" << std::endl;

    //delete index2_cellvec;
    std::cout << "-8" << std::endl;

    }
}
