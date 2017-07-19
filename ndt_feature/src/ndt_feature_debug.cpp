// #include <iostream>
// #include <boost/program_options.hpp>
// #include <boost/math/distributions/normal.hpp>
// #include <boost/random.hpp>
// #include <boost/random/uniform_real.hpp>
// #include <boost/random/variate_generator.hpp>
// 
// #include <ndt_registration/ndt_matcher_d2d.h>
// #include <ndt_registration/ndt_matcher_d2d_feature.h>
// #include <ndt_feature/ndt_matcher_d2d_fusion.h>
// #include <ndt_map/ndt_map.h>
// #include <ndt_map/cell_vector.h>
// #include <ndt_map/lazy_grid.h>
// #include <ndt_map/pointcloud_utils.h>
// 
// #include <ndt_feature/ndt_rviz.h>
// #include <ros/ros.h>
// 
// #include <ndt_feature/utils.h>
// 
// using namespace std;
// using namespace lslgeneric;
// namespace po = boost::program_options;
// 
// 
// 
// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "ndt_feature_test");
// 
//     cout << "--------------------------------------------------" << endl;
//     cout << "Small test program of CellVector + F2F matcher " << endl;
//     cout << "--------------------------------------------------" << endl;
// 
// 
//     po::options_description desc("Allowed options");
//     Eigen::Matrix<double,6,1> pose_increment_v;
//     string static_file_name, moving_file_name;
//     int nb_clusters, nb_points;
//     double std_dev, min, max;
//     desc.add_options()
//     ("help", "produce help message")
//     ("x", po::value<double>(&pose_increment_v(0))->default_value(1.), "x pos gt offset")
//     ("y", po::value<double>(&pose_increment_v(1))->default_value(1.), "y pos gt offset")
//     ("z", po::value<double>(&pose_increment_v(2))->default_value(1.), "z pos gt offset")
//     ("X", po::value<double>(&pose_increment_v(3))->default_value(0.1), "x axis rot gt offset")
//     ("Y", po::value<double>(&pose_increment_v(4))->default_value(0.1), "y axis rot gt offset")
//     ("Z", po::value<double>(&pose_increment_v(5))->default_value(0.1), "z axis rot gt offset")
//     ("nb_clusters", po::value<int>(&nb_clusters)->default_value(20), "number of clusters")
//     ("nb_points", po::value<int>(&nb_points)->default_value(10), "number of points per clusters")
//     ("std_dev", po::value<double>(&std_dev)->default_value(0.1), "standard deviation of the points drawn from a normal distribution (here independent on the axes")
//     ("min", po::value<double>(&min)->default_value(-10), "minimum center point")
//     ("max", po::value<double>(&max)->default_value(10), "maximum center point")
//     ("p2preg", "calculate match using two pointclouds")
//     ("irregular_grid", "use irregular grid in the p2p registration")
//     ("nfeaturecorr", "feature_f2f should NOT be evaluated (NDTMatcherFeatureF2F)")
//     ("singleres", "use single resolution in the 'p2p' matching")
//     ("nf2f", "if f2f should NOT be evaluated")
//     ("usegt", "if the ground truth should be used as initial estimate")
//     ("nuse_fusion", "if fusion matching should *NOT* be used")
//     ("empty_corr", "if not corerspondances should be used - test the fusion without features")
//     ("empty_ndt", "if empty ndt map should be used - test the fusion without the normal ndts")
//     ("nuse_NDT", "fusion matching debug param : if NDT should *NOT* be used")
//     ("nuse_feat", "fusion matching debug param : if features should *NOT* be used")
//     ("nstep_control", "fusion matching debug param : if step_control should *NOT* be used")
//     ("init_T_with_corr", "use the correspondance vec + mean value to compute a rigid transformation")
//     ;
// 
//     po::variables_map vm;
//     po::store(po::parse_command_line(argc, argv, desc), vm);
//     po::notify(vm);
// 
//     if (vm.count("help"))
//     {
//         cout << desc << "\n";
//         return 1;
//     }
// 
//     ros::NodeHandle nh;
//     ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
// 
//     bool use_p2preg = vm.count("p2preg");
//     bool use_irregular_grid = vm.count("irregular_grid");
//     bool use_featurecorr = !vm.count("nfeaturecorr");
//     bool use_singleres = vm.count("singleres");
//     bool use_f2f = !vm.count("nf2f");
//     bool usegt = vm.count("usegt");
//     bool use_fusion = !vm.count("nuse_fusion");
//     bool empty_corr = vm.count("empty_corr");
//     bool empty_ndt = vm.count("empty_ndt");
//     bool use_NDT = !vm.count("nuse_NDT");
//     bool use_feat = !vm.count("nuse_feat");
//     bool step_control = !vm.count("nstep_control");
//     bool use_d2d = true;
//     bool use_ransac = vm.count("use_ransac");
//     bool init_T_with_corr = vm.count("init_T_with_corr");
// 
//     pcl::PointCloud<pcl::PointXYZ> static_pc, moving_pc;
//     Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> gt_transform;
//     Eigen::MatrixXd T_cov(6,6);
//     T_cov.setIdentity();
// 
//     // Generate the static point cloud + indices...
//     boost::mt19937 rng;
//     boost::uniform_real<> ud(min,max);
//     boost::normal_distribution<> nd(0.0, std_dev);
//     boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);
//     boost::variate_generator<boost::mt19937&, boost::uniform_real<> > var_uni(rng, ud);
//     std::vector<std::pair<int,int> > corresp;
// 
//     size_t idx = 0;
//     std::vector<std::vector<size_t> > all_indices;
//     for (int i = 0; i < nb_clusters; i++)
//     {
//         std::vector<size_t> indices;
//         double c_x = var_uni();
//         double c_y = var_uni();
//         double c_z = var_uni();
//         for (int j = 0; j < nb_points; j++)
//         {
//             static_pc.push_back(pcl::PointXYZ(c_x+var_nor(), c_y+var_nor(), c_z+var_nor()));
//             indices.push_back(idx);
//             idx++;
//         }
//         all_indices.push_back(indices);
//         corresp.push_back(std::pair<int,int>(i,nb_clusters-1-i)); // nb_clusters-1-i -> To check the da functions is working.
//     }
// 
//     std::vector<std::vector<size_t> > all_indices_moving(all_indices.size());  // Reverse the correspondances (To check the da functions)
//     std::reverse_copy(all_indices.begin(), all_indices.end(), all_indices_moving.begin());
// 
//     // Specify some offset...
//     gt_transform = Eigen::Translation<double,3>(pose_increment_v(0),pose_increment_v(1),pose_increment_v(2))*
//                    Eigen::AngleAxis<double>(pose_increment_v(3),Eigen::Vector3d::UnitX()) *
//                    Eigen::AngleAxis<double>(pose_increment_v(4),Eigen::Vector3d::UnitY()) *
//                    Eigen::AngleAxis<double>(pose_increment_v(5),Eigen::Vector3d::UnitZ()) ;
// 
//     moving_pc = lslgeneric::transformPointCloud<pcl::PointXYZ>(gt_transform, static_pc);
//     // Generation done.
// 
//     Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T_f2f,T_p2p, T_feat, T_fusion, T_d2d, T_corr;
//     T_f2f.setIdentity();
//     T_p2p.setIdentity();
//     T_feat.setIdentity();
//     T_fusion.setIdentity();
//     T_d2d.setIdentity();
//     T_corr.setIdentity();
//     if (usegt)
//     {
//         T_f2f = gt_transform;
//         T_p2p = gt_transform;
//         T_feat = gt_transform;
//         T_fusion = gt_transform;
//         T_d2d = gt_transform;
//     }
// 
//     std::vector<double> resolutions;
//     if (use_singleres)
//         resolutions.push_back(1.);
//     NDTMatcherD2D matcher(use_irregular_grid,!use_singleres,resolutions);
//     
//     if (use_p2preg)
//     {
//         matcher.match(moving_pc, static_pc, T_p2p);
//     }
// 
//     double current_resolution = 1;
//     
//     SpatialIndex* index1_lazzy = NULL;
//     SpatialIndex* index2_lazzy = NULL;
//     SpatialIndex* index1_cellvec = NULL;
//     SpatialIndex* index2_cellvec = NULL;
//         
//     index1_lazzy = new LazyGrid(current_resolution);
//     index2_lazzy = new LazyGrid(current_resolution);
//     index1_cellvec = new CellVector();
//     index2_cellvec = new CellVector();
//     
//     std::cout << "before creating static NDTMap" << std::endl;
//     NDTMap ndt(index1_lazzy);
//     ndt.loadPointCloud (static_pc );
//     ndt.computeNDTCells();
//     NDTMap ndt_feat(index1_cellvec);
//     ndt_feat.loadPointCloud(static_pc, all_indices); 
//     ndt_feat.computeNDTCellsSimple();
//     std::cerr << "static numberOfActiveCells1 : " << ndt.numberOfActiveCells() << std::endl;
//     
//     std::cout << "before creating moving NDTMap" << std::endl;
//     NDTMap mov(index2_lazzy);
//     mov.loadPointCloud( moving_pc );
//     mov.computeNDTCells();
//     NDTMap mov_feat(index2_cellvec);
//     mov_feat.loadPointCloud(moving_pc, all_indices_moving); 
//     mov_feat.computeNDTCellsSimple();
// 
//     std::cerr << "static cloud index str : " << ndt.getMyIndexStr() << std::endl;
//     std::cerr << "moving cloud index str : " << mov.getMyIndexStr() << std::endl;
// 
//     std::cerr << "static - feature index str : " << ndt_feat.getMyIndexStr() << std::endl;
//     std::cerr << "moving - feature index str : " << mov_feat.getMyIndexStr() << std::endl;
//     
//     std::cerr << "static numberOfActiveCells : " << ndt.numberOfActiveCells() << std::endl;
//     std::cerr << "moving numberOfActiveCells : " << mov.numberOfActiveCells() << std::endl;
//     
//     std::cerr << "all_indices.size() : " << all_indices.size() << std::endl;
//     std::cerr << "all_indices_moving.size() : " << all_indices_moving.size() << std::endl;
//     
//     std::cerr << "corresp.size() : " << corresp.size() << std::endl;
//     
//     std::cerr << "moving_pc.size() : " << moving_pc.size() << std::endl;
//     
//     T_corr = ICPwithCorrMatch(mov_feat, ndt_feat, corresp);
//     if (init_T_with_corr) {
//       T_f2f = T_corr;
//       T_p2p = T_corr;
//       T_feat = T_corr;
//       T_fusion = T_corr;
//       T_d2d = T_corr;
//     }
//     
//     if (use_f2f)
//     {
//       matcher.match( mov_feat, ndt_feat, T_f2f );
//     }
//     if (use_featurecorr)
//     {
//       std::cout << "------------- MATCHING featured2d ------------------" << std::endl;
//       std::cout << "use_featurecorr!" << std::endl;
//       NDTMatcherFeatureD2D matcher_feat(corresp);
//       std::cerr << "MATCHING START" << std::endl;
//       matcher_feat.match( mov_feat, ndt_feat, T_feat );
//       std::cerr << "MATCHING ENDSTART" << std::endl;
//     }
//     //        delete index;
// //    }
// 
// 
//     if (use_d2d)
//     {
//       std::cout << "------------- MATCHING d2d ------------------" << std::endl;
//       matcher.match( mov, ndt, T_d2d );
//     }
// 
//     if (use_fusion) {
//       std::vector<std::pair<int, int> > corresp2;
//       if (!empty_corr)
//         corresp2=corresp;
//       std::cout << "------------ MATCHING matchFUsion -------------" << std::endl;;
//       std::cout << "ndt.numberOfActiveCells() : " << ndt.numberOfActiveCells() << std::endl;
//       std::cout << "mov.numberOfActiveCells() : " << mov.numberOfActiveCells() << std::endl;
//       matchFusion(mov, ndt,
//                   mov_feat, ndt_feat,
//                   corresp2,
//                   T_fusion,
//                   T_cov,
//                   true,
//                   use_NDT,
//                   use_feat,
//                   step_control,
//                   100);
//     }
// 
//     std::cout<<"GT translation "<<gt_transform.translation().transpose()
//              <<" (norm) "<<gt_transform.translation().norm()<<std::endl;
//     std::cout<<"GT rotation "<<gt_transform.rotation().eulerAngles(0,1,2).transpose()
//              <<" (norm) "<<gt_transform.rotation().eulerAngles(0,1,2).norm()<<std::endl;
// 
//     std::cout<<"T_corr (icp with corr) translation "<<T_corr.translation().transpose()
//              <<" (norm) "<<T_corr.translation().norm()<<std::endl;
//     std::cout<<"T_corr (icp with corr) rotation "<<T_corr.rotation().eulerAngles(0,1,2).transpose()
//              <<" (norm) "<<T_corr.rotation().eulerAngles(0,1,2).norm()<<std::endl;
// 
// 
//     if (use_f2f)
//     {
//         std::cout<<"f2f translation "<<T_f2f.translation().transpose()
//                  <<" (norm) "<<T_f2f.translation().norm()<<std::endl;
//         std::cout<<"f2f rotation "<<T_f2f.rotation().eulerAngles(0,1,2).transpose()
//                  <<" (norm) "<<T_f2f.rotation().eulerAngles(0,1,2).norm()<<std::endl;
//     }
// 
//     if (use_featurecorr)
//     {
//         std::cout<<"feat w. corr. translation "<<T_feat.translation().transpose()
//                  <<" (norm) "<<T_feat.translation().norm()<<std::endl;
//         std::cout<<"feat w. corr. rotation "<<T_feat.rotation().eulerAngles(0,1,2).transpose()
//                  <<" (norm) "<<T_feat.rotation().eulerAngles(0,1,2).norm()<<std::endl;
//     }
//     if (use_fusion)
//     {
//         std::cout<<"fusion translation "<<T_fusion.translation().transpose()
//                  <<" (norm) "<<T_fusion.translation().norm()<<std::endl;
//         std::cout<<"fusion rotation "<<T_fusion.rotation().eulerAngles(0,1,2).transpose()
//                  <<" (norm) "<<T_fusion.rotation().eulerAngles(0,1,2).norm()<<std::endl;
//     }
//     if (use_d2d)
//     {
//         std::cout<<"d2d translation "<<T_d2d.translation().transpose()
//                  <<" (norm) "<<T_d2d.translation().norm()<<std::endl;
//         std::cout<<"d2d rotation "<<T_d2d.rotation().eulerAngles(0,1,2).transpose()
//                  <<" (norm) "<<T_d2d.rotation().eulerAngles(0,1,2).norm()<<std::endl;
//     }
//     if (use_p2preg)
//     {
//         std::cout<<"p2preg translation "<<T_p2p.translation().transpose()
//                  <<" (norm) "<<T_p2p.translation().norm()<<std::endl;
//         std::cout<<"p2preg rotation "<<T_p2p.rotation().eulerAngles(0,1,2).transpose()
//                  <<" (norm) "<<T_p2p.rotation().eulerAngles(0,1,2).norm()<<std::endl;
//     }
//     cout << "done." << endl;
// 
//     ros::Rate loop_rate(1);
// 
//     while (ros::ok())
//     {
//         marker_pub.publish(ndt_visualisation::markerNDTCells(ndt_feat, 1));
//         marker_pub.publish(ndt_visualisation::markerNDTCells(mov_feat, 2));
//         marker_pub.publish(ndt_visualisation::markerCellVectorCorrespondances(ndt_feat, mov_feat, corresp));
// 
//         marker_pub.publish(ndt_visualisation::markerNDTCells(ndt, 3));
//         marker_pub.publish(ndt_visualisation::markerNDTCells(mov, 4));
//         
//         ros::spinOnce();
//         loop_rate.sleep();
//     }
// 
//     delete index1_lazzy;
//     delete index2_lazzy;
//     delete index1_cellvec;
//     delete index2_cellvec;
// }
