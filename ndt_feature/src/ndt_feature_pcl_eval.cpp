// It works if the --p2p option is given!??

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

#include <ndt_feature/ndt_rviz.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/point_tests.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/shot.h>
#include <pcl/features/shot_omp.h>
#include <pcl/PointIndices.h>

using namespace std;
using namespace lslgeneric;
namespace po = boost::program_options;

float support_size = 0.2f;
bool rotation_invariant = false; //true;
bool add_points_on_straight_edges = false;
float distance_for_additional_points = 0.5;


template<class T> std::string toString (const T& x)
{
     std::ostringstream o;

     if (!(o << x))
	  throw std::runtime_error ("::toString()");

     return o.str ();
}

Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> ICPwithCorrMatch(NDTMap &targetNDT, NDTMap &sourceNDT, const std::vector<std::pair<int,int> > &corresp) {
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T;
    T.setIdentity();

    Eigen::Vector3d c0;
    Eigen::Vector3d c1;
    Eigen::Matrix3d H;
    {
        c0.setZero();
        c1.setZero();

        std::cout << "1" << std::endl;
        
        // get centroids
        for (size_t i = 0; i < corresp.size(); i++) {
            c0 += targetNDT.getCellIdx(corresp[i].first)->getMean();
            c1 += sourceNDT.getCellIdx(corresp[i].second)->getMean();
        }
        c0 *= (1./(1.*corresp.size()));
        c1 *= (1./(1.*corresp.size()));
        std::cout << "1" << std::endl;
        std::cout << "c0 : " << c0 << std::endl;
        std::cout << "c1 : " << c1 << std::endl;
        
        Eigen::MatrixXd target(corresp.size(), 3);
        std::cout << "target : " << target << std::endl;
        Eigen::MatrixXd source(corresp.size(), 3);
        for (size_t i = 0; i < corresp.size(); i++) {
            target.row(i) = targetNDT.getCellIdx(corresp[i].first)->getMean() - c0;
            source.row(i) = sourceNDT.getCellIdx(corresp[i].second)->getMean() - c1;
        }

        std::cout << "target : " << target << std::endl; 
        std::cout << "source : " << source << std::endl;
        std::cout << "target * source.transpose() : " << target  * source.transpose() << std::endl;

        H = target.transpose() * source;

        std::cout << "H : " << H << std::endl;
    }
    
    // do the SVD thang
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d R = V * svd.matrixU().transpose();
    double det = R.determinant();
    //ntot++;
    if (det < 0.0)
    {
        //nneg++;
        V.col(2) = V.col(2)*-1.0;
        R = V * svd.matrixU().transpose();
    }
    Eigen::Vector3d tr = c0-R.transpose()*c1;    // translation
    
    std::cout << "translation : " << c0 - c1 << std::endl;
    std::cout << "tr : " << tr << std::endl;
    std::cout << "c0-R.transpose()*c1 : " << c0-R.transpose()*c1 << std::endl;
    // transformation matrix, 3x4
    Eigen::Matrix<double,3,4> tfm;
    tfm.block<3,3>(0,0) = R.inverse();
    tfm.col(3) = tr;
    T = tfm;

    return T;
}


// Didn't manage to figure out how the ASCIIReader really worked quick enough...
pcl::PointCloud<pcl::PointXYZ> readCSVFile(const std::string &fileName) {
    std::ifstream ifs;
    ifs.open(fileName.c_str());
    if (!ifs.is_open()) {
        std::cerr << __FILE__ << ":" << __LINE__ << " cannot open file : " << fileName << std::endl;
    }
    
    pcl::PointCloud<pcl::PointXYZ> pc;
    
    std::string line;
    getline(ifs, line); // Skip the first line...
    
    while (!ifs.eof())
    {
        getline(ifs, line);
        
        double time,x,y,z,i;
        if (sscanf(line.c_str(), "%lf,%lf,%lf,%lf,%lf",
                   &time,&x,&y,&z,&i) == 5)
        {
            pcl::PointXYZ pt;
            pt.x = x; pt.y = y; pt.z = z; //pt.intensity = i;
            pc.push_back(pt);
        }
    }
    return pc;
}

pcl::RangeImage computeRangeImage(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
    // Parameters needed by the range image object:
    
    // Angular resolution is the angular distance between pixels.
    // Kinect: 57° horizontal FOV, 43° vertical FOV, 640x480 (chosen here).
    // Xtion: 58° horizontal FOV, 45° vertical FOV, 640x480.
//    float angularResolutionX = (float)(57.0f / 640.0f * (M_PI / 180.0f));
//    float angularResolutionY = (float)(43.0f / 480.0f * (M_PI / 180.0f));
    float angularResolutionX = (float)(0.4 * (M_PI / 180.0f));
    float angularResolutionY = (float)(0.4 * (M_PI / 180.0f));

    // Maximum horizontal and vertical angles. For example, for a full panoramic scan,
    // the first would be 360º. Choosing values that adjust to the real sensor will
    // decrease the time it takes, but don't worry. If the values are bigger than
    // the real ones, the image will be automatically cropped to discard empty zones.
//    float maxAngleX = (float)(60.0f * (M_PI / 180.0f));
//    float maxAngleY = (float)(50.0f * (M_PI / 180.0f));
    float maxAngleX = (float)(270.0f * (M_PI / 180.0f));
    float maxAngleY = (float)(140.0f * (M_PI / 180.0f));

    // Sensor pose. Thankfully, the cloud includes the data.
    Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud.sensor_origin_[0],
                                                                      cloud.sensor_origin_[1],
                                                                      cloud.sensor_origin_[2])) *
        Eigen::Affine3f(cloud.sensor_orientation_);
    // Noise level. If greater than 0, values of neighboring points will be averaged.
    // This would set the search radius (i.e., 0.03 == 3cm).
    float noiseLevel = 0.03f;
    // Minimum range. If set, any point closer to the sensor than this will be ignored.
    float minimumRange = 1.f;
    // Border size. If greater than 0, a border of "unobserved" points will be left
    // in the image when it is cropped.
    int borderSize = 1;
    
    // Range image object.
    pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(cloud, angularResolutionX, angularResolutionY,
                                    maxAngleX, maxAngleY, sensorPose, pcl::RangeImage::CAMERA_FRAME,
                                    noiseLevel, minimumRange, borderSize);
    return rangeImage;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> dualVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr static_pc, pcl::PointCloud<pcl::PointXYZ>::ConstPtr moving_pc)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (static_pc, "static_pc");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "static_pc");
  viewer->addPointCloud<pcl::PointXYZ> (moving_pc, "moving_pc");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1., 0., 0., "moving_pc");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

pcl::PointIndicesConstPtr getPCLPointIndices(pcl::PointCloud<int> &indices) {
    
    pcl::PointIndicesPtr ids(new pcl::PointIndices());
    for (size_t i = 0; i < indices.size(); i++) {
        ids->indices.push_back(indices[i]);
    }
    return ids;
}

void extractNARFkeypoints(pcl::RangeImage &range_image, pcl::PointCloud<int> &keypoint_indices) {
    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage (&range_image);
    narf_keypoint_detector.getParameters ().support_size = support_size;
    narf_keypoint_detector.getParameters ().add_points_on_straight_edges = add_points_on_straight_edges;
    narf_keypoint_detector.getParameters ().distance_for_additional_points = distance_for_additional_points;
    
    narf_keypoint_detector.compute (keypoint_indices);
    std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";
}

void extractNARFdescriptor(pcl::RangeImage &range_image, pcl::PointCloud<int> &keypoint_indices, pcl::PointCloud<pcl::Narf36> &descriptors) {
  std::vector<int> keypoint_indices2;
  keypoint_indices2.resize (keypoint_indices.points.size ());
  for (unsigned int i=0; i<keypoint_indices.size (); ++i) // This step is necessary to get the right vector type
    keypoint_indices2[i]=keypoint_indices.points[i];
  pcl::NarfDescriptor narf_descriptor (&range_image, &keypoint_indices2);
  narf_descriptor.getParameters ().support_size = support_size;
  narf_descriptor.getParameters ().rotation_invariant = rotation_invariant;
  //pcl::PointCloud<pcl::Narf36> narf_descriptors;
  narf_descriptor.compute (descriptors);
  cout << "Extracted "<<descriptors.size ()<<" descriptors for "
                      <<keypoint_indices.points.size ()<< " keypoints.\n";
}

void computeNormals(pcl::PointCloud<pcl::PointXYZ> &pc, pcl::PointCloud<pcl::Normal> &normals) {
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setKSearch (10);
    norm_est.setInputCloud (pc.makeShared());
    norm_est.compute (normals);

    std::cout << "Computed " << normals.size() << " normals." << std::endl;
}

void extractSamplingKeypoints(pcl::PointCloud<pcl::PointXYZ> &pc, pcl::PointCloud<int> &keypoint_indices) {
    
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud (pc.makeShared());
    uniform_sampling.setRadiusSearch (0.4);
    uniform_sampling.compute (keypoint_indices);

    std::cout << "Detected " << keypoint_indices.size() << " keypoints." << std::endl;

}

void extractSHOTdescriptor(pcl::PointCloud<pcl::PointXYZ> &pc, pcl::PointCloud<pcl::Normal> &normals, pcl::PointCloud<int> &keypoint_indices, pcl::PointCloud<pcl::SHOT352> &descriptors) {
    pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352>  descr_est;
    descr_est.setRadiusSearch(0.4f);
    descr_est.setInputNormals(normals.makeShared());
//    descr_est.setSearchSurface(pc.makeShared());
    descr_est.setInputCloud(pc.makeShared());
    descr_est.setIndices(getPCLPointIndices(keypoint_indices));
    descr_est.compute(descriptors);

    std::cout << "Computed " << descriptors.size() << " SHOT descriptors." << std::endl;
}

void featureCorrespondencesNARF(pcl::PointCloud<pcl::Narf36> &static_desc, pcl::PointCloud<pcl::Narf36> &moving_desc, std::vector<std::pair<int, int> > &correspondences_out, std::vector<float> &correspondence_scores_out) {
    
    correspondences_out.resize(moving_desc.size());
    correspondence_scores_out.resize(moving_desc.size());
    
    // Use a KdTree to search for the nearest matches in feature space
    pcl::search::KdTree<pcl::Narf36> descriptorkdtree;
    descriptorkdtree.setInputCloud(static_desc.makeShared());
    // Find the index of the best match for each keypoint
    const int k = 1;
    std::vector<int> k_indices(k) ;
    std::vector<float> k_squared_distances(k);
    
    for (size_t i = 0; i < moving_desc.size(); ++i) {
        descriptorkdtree.nearestKSearch(moving_desc, i, k, k_indices, k_squared_distances);
//        if (k_squared_distances[0] < 0.05) {
            correspondences_out[i] = std::pair<int, int>(k_indices[0], i);
            correspondence_scores_out[i] = k_squared_distances[0];
//        }
    }
}

#if 0
bool isSHOT352Valid(const pcl::SHOT352 &desc) {
    for (size_t i = 0; i < 352; i++) {
        if (!pcl::isFinite(desc.descriptor[i]))
            return false;
    }
    return true;
}
#endif

void featureCorrespondences(pcl::PointCloud<pcl::SHOT352> &static_desc, pcl::PointCloud<pcl::SHOT352> &moving_desc, pcl::Correspondences &correspondences)
 {
    correspondences.resize(static_desc.size());
    pcl::search::KdTree<pcl::SHOT352> descriptorkdtree;
    descriptorkdtree.setInputCloud(moving_desc.makeShared());
    // Find the index of the best match for each keypoint
    const int k = 1;
    std::vector<int> k_indices(k) ;
    std::vector<float> k_squared_distances(k);
    
    for (size_t i = 0; i < static_desc.size(); ++i) {
        if (pcl::isFinite<pcl::SHOT352>(static_desc[i])) {
            descriptorkdtree.nearestKSearch(static_desc, i, k, k_indices, k_squared_distances);
            correspondences.push_back(pcl::Correspondence(i, k_indices[0], k_squared_distances[0]));
        }
        else {
            std::cerr << "invalid point detected!!!" << std::endl;
        }
    }
}


pcl::PointXYZ getPointXYZFromDescriptor(const pcl::Narf36 &desc) {
    pcl::PointXYZ pt;
    pt.x = desc.x; pt.y = desc.y; pt.z = desc.z;
    return pt;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ndt_feature_test");

    cout << "---------------------------------------------------------------------------------------------" << endl;
    cout << "Evaluation program. Put in two .cvs point cloud files and get the Transformation matrix (4x4)" << endl;
    cout << "---------------------------------------------------------------------------------------------" << endl;

    po::options_description desc("Allowed options");
    Eigen::Matrix<double,6,1> pose_increment_v;
    string static_file_name, moving_file_name;
    int nb_clusters, nb_points;
    double std_dev, min, max;
    desc.add_options()
        ("help", "produce help message")
        ("static", po::value<std::string>(&static_file_name)->default_value(std::string("")), "filename of the static point cloud")
        ("moving", po::value<std::string>(&moving_file_name)->default_value(std::string("")), "filename of the moving point cloud")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        cout << desc << "\n";
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZ> static_pc = readCSVFile(static_file_name);
    pcl::PointCloud<pcl::PointXYZ> moving_pc = readCSVFile(moving_file_name);
    
    // Save the pc...
    pcl::io::savePCDFileASCII ("static.pcd", static_pc);
    pcl::io::savePCDFileASCII ("moving.pcd", moving_pc);

    std::cout << "Loaded #" << static_pc.size() << " points from " << static_file_name << std::endl;
    std::cout << "Loaded #" << moving_pc.size() << " points from " << moving_file_name << std::endl;

    // Compute range images
    pcl::RangeImage range_image_static = computeRangeImage(static_pc);
    boost::shared_ptr<pcl::RangeImage> range_image_static_ptr = range_image_static.makeShared();

    pcl::RangeImage range_image_moving = computeRangeImage(moving_pc);
    boost::shared_ptr<pcl::RangeImage> range_image_moving_ptr = range_image_moving.makeShared();


    
  pcl::PointCloud<int> keypoint_indices_static;
  pcl::PointCloud<int> keypoint_indices_moving;

  // --------------------------------
  // -----Extract NARF-----
  // --------------------------------
  pcl::PointCloud<pcl::Narf36> narf_descriptors_static;
  pcl::PointCloud<pcl::Narf36> narf_descriptors_moving;

  extractNARFkeypoints(range_image_static, keypoint_indices_static);
  extractNARFdescriptor(range_image_static, keypoint_indices_static, narf_descriptors_static);

  extractNARFkeypoints(range_image_moving, keypoint_indices_moving);
  extractNARFdescriptor(range_image_moving, keypoint_indices_moving, narf_descriptors_moving);


  // ------------------------------
  // -----Subsampled keypoints-----
  // ------------------------------
  extractSamplingKeypoints(static_pc, keypoint_indices_static);
  extractSamplingKeypoints(moving_pc, keypoint_indices_moving);

  
  // --------------------
  // ----Extract SHOT----
  // -------------------
  pcl::PointCloud<pcl::Normal> static_normals;
  pcl::PointCloud<pcl::Normal> moving_normals;
  computeNormals(static_pc, static_normals);
  std::cout << "static_normals : " << static_normals.size() << std::endl;
  computeNormals(moving_pc, moving_normals);
  
  pcl::PointCloud<pcl::SHOT352> shot_descriptors_static;
  pcl::PointCloud<pcl::SHOT352> shot_descriptors_moving;

  extractSHOTdescriptor(static_pc, static_normals, keypoint_indices_static, shot_descriptors_static);
  extractSHOTdescriptor(static_pc, static_normals, keypoint_indices_moving, shot_descriptors_moving);

  // Check that they seems to be ok...
  for (size_t i = 0; i < shot_descriptors_static.size(); ++i) {
      if (!pcl::isFinite<pcl::SHOT352>(shot_descriptors_static[i])) {
          std::cerr << "invalid static descriptor at " << i << std::endl;
      }
  }
  for (size_t i = 0; i < shot_descriptors_moving.size(); ++i) {
      if (!pcl::isFinite<pcl::SHOT352>(shot_descriptors_moving[i])) {
          std::cerr << "invalid moving descriptor at " << i << std::endl;
      }
      for (size_t j = 0; j < 352; j++) {
          if (std::isnan(shot_descriptors_moving[i].descriptor[j]))
              std::cerr << "aösdlkfjaölskdjf" << std::endl;
      }
  }



#if 0
  // ---------------------------------
  // -----Compute correspondances-----
  // ---------------------------------
  std::vector<std::pair<int, int> > correspondences;
  std::vector<float> correspondence_scores;
  featureCorrespondencesNARF(narf_descriptors_static, narf_descriptors_moving, correspondences, correspondence_scores);
#endif
  pcl::Correspondences correspondences;
  featureCorrespondences(shot_descriptors_static, shot_descriptors_moving, correspondences);

  // ----------------------
  // -----Registration-----
  // ----------------------


  // ------------------------
  // -----Visualizations-----
  // ------------------------
  pcl::visualization::RangeImageVisualizer range_image_viewer_static("Range image static");
  range_image_viewer_static.showRangeImage(range_image_static);
  pcl::visualization::RangeImageVisualizer range_image_viewer_moving("Range image moving");
  range_image_viewer_moving.showRangeImage(range_image_moving);

  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor (1, 1, 1);
  {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_static_ptr, 0, 0, 0);
      viewer.addPointCloud (range_image_static_ptr, range_image_color_handler, "range image static");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image static");
  }
  {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_moving_ptr, 100, 0, 100);
      viewer.addPointCloud (range_image_moving_ptr, range_image_color_handler, "range image moving");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image moving");
  }
  viewer.initCameraParameters ();

  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_static_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>& keypoints_static = *keypoints_static_ptr;

  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_moving_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>& keypoints_moving = *keypoints_moving_ptr;
  
  bool use_range_image = false;
  {
      keypoints_static.points.resize (keypoint_indices_static.points.size ());
      for (size_t i=0; i<keypoint_indices_static.points.size (); ++i) {
          if (use_range_image)
              keypoints_static.points[i].getVector3fMap () = range_image_static.points[keypoint_indices_static.points[i]].getVector3fMap ();
          else
              keypoints_static.points[i] = static_pc[keypoint_indices_static.points[i]];
      }
      
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints_static_ptr, 0, 255, 0);
      viewer.addPointCloud<pcl::PointXYZ> (keypoints_static_ptr, keypoints_color_handler, "keypoints static");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints static");
  }
  {
      keypoints_moving.points.resize (keypoint_indices_moving.points.size ());
      for (size_t i=0; i<keypoint_indices_moving.points.size (); ++i) {
          if (use_range_image)
              keypoints_moving.points[i].getVector3fMap () = range_image_moving.points[keypoint_indices_moving.points[i]].getVector3fMap ();
          else
              keypoints_moving.points[i] = moving_pc[keypoint_indices_moving.points[i]];
      }
      
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints_moving_ptr, 0, 0, 255);
      viewer.addPointCloud<pcl::PointXYZ> (keypoints_moving_ptr, keypoints_color_handler, "keypoints moving");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints moving");
  }

#if 0
  {
      // Draw correspondances
      std::cout << "Number of correspondences : " << correspondences.size() << std::endl;
      for (size_t i = 0; i < correspondences.size(); ++i) {
          std::string name = "line" + toString(i);
          viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(getPointXYZFromDescriptor(narf_descriptors_static[correspondences[i].first]), 
                                                       getPointXYZFromDescriptor(narf_descriptors_moving[correspondences[i].second]),
                                                       name );
          std::cout << correspondences[i].first << "<->" << correspondences[i].second << ", score: " << correspondence_scores[i] << " x: " << getPointXYZFromDescriptor(narf_descriptors_static[correspondences[i].first]) << std::endl;
      }
  }
#endif

  {
      // Draw correspondances
      std::cout << "Number of correspondences : " << correspondences.size() << std::endl;
      for (size_t i = 0; i < correspondences.size(); ++i) {
          std::string name = "line" + toString(i);
          viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(static_pc[correspondences[i].index_query], 
                                                       moving_pc[correspondences[i].index_match],
                                                       name );
          std::cout << correspondences[i].index_query << "<->" << correspondences[i].index_match << ", score: " << correspondences[i].distance << std::endl;
      }
  }

  
  while (!viewer.wasStopped ())
  {
    range_image_viewer_static.spinOnce ();
    range_image_viewer_moving.spinOnce ();
    viewer.spinOnce ();
    pcl_sleep(0.01);
  }


#if 0
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    bool use_lazzy = vm.count("lazzy");
    bool use_p2preg = vm.count("p2preg");
    bool use_irregular_grid = vm.count("irregular_grid");
    bool use_featurecorr = !vm.count("nfeaturecorr");
    bool use_singleres = vm.count("singleres");
    bool use_f2f = !vm.count("nf2f");
    bool usegt = vm.count("usegt");
    bool use_fusion = vm.count("use_fusion");
    bool empty_corr = vm.count("empty_corr");
    bool empty_ndt = vm.count("empty_ndt");
    bool use_NDT = vm.count("use_NDT");
    bool use_feat = vm.count("use_feat");
    bool step_control = vm.count("step_control");
    bool use_d2d = true;
    bool use_ransac = vm.count("use_ransac");
    bool init_T_with_corr = vm.count("init_T_with_corr");

    pcl::PointCloud<pcl::PointXYZ> static_pc, moving_pc, tmp_pc;
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> gt_transform;

    // Generate the static point cloud + indices...
    boost::mt19937 rng;
    boost::uniform_real<> ud(min,max);
    boost::normal_distribution<> nd(0.0, std_dev);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > var_uni(rng, ud);
    std::vector<std::pair<int,int> > corresp;

    size_t idx = 0;
    std::vector<std::vector<size_t> > all_indices;
    for (int i = 0; i < nb_clusters; i++)
    {
        std::vector<size_t> indices;
        double c_x = var_uni();
        double c_y = var_uni();
        double c_z = var_uni();
        for (int j = 0; j < nb_points; j++)
        {
            static_pc.push_back(pcl::PointXYZ(c_x+var_nor(), c_y+var_nor(), c_z+var_nor()));
            indices.push_back(idx);
            idx++;
        }
        all_indices.push_back(indices);
        corresp.push_back(std::pair<int,int>(i,nb_clusters-1-i)); // nb_clusters-1-i -> To check the da functions is working.
    }
    std::vector<std::vector<size_t> > all_indices_moving(all_indices.size());  // Reverse the correspondances (To check the da functions)
    std::reverse_copy(all_indices.begin(), all_indices.end(), all_indices_moving.begin());

    for (int i = 0; i < 1/*nb_clusters*/; i++)
    {
        tmp_pc.push_back(pcl::PointXYZ(i,i,i));
    }

    // Specify some offset...
    gt_transform = Eigen::Translation<double,3>(pose_increment_v(0),pose_increment_v(1),pose_increment_v(2))*
                   Eigen::AngleAxis<double>(pose_increment_v(3),Eigen::Vector3d::UnitX()) *
                   Eigen::AngleAxis<double>(pose_increment_v(4),Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxis<double>(pose_increment_v(5),Eigen::Vector3d::UnitZ()) ;


    std::vector<double> resolutions;
    if (use_singleres)
        resolutions.push_back(1.);
    NDTMatcherD2D matcher(use_irregular_grid,!use_singleres,resolutions);
    NDTMatcherD2D matcher2(use_irregular_grid,!use_singleres,resolutions);
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T_f2f,T_p2p, T_feat, T_fusion, T_d2d, T_corr;
    T_f2f.setIdentity();
    T_p2p.setIdentity();
    T_feat.setIdentity();
    T_fusion.setIdentity();
    T_d2d.setIdentity();
    T_corr.setIdentity();
    if (usegt)
    {
        T_f2f = gt_transform;
        T_p2p = gt_transform;
        T_feat = gt_transform;
        T_fusion = gt_transform;
        T_d2d = gt_transform;
    }

    moving_pc = transformPointCloud(gt_transform, static_pc);

    if (use_p2preg)
    {
        matcher.match(moving_pc, static_pc, T_p2p);
    }
//    {
        double current_resolution = 1;

        SpatialIndex* index1 = NULL;
        SpatialIndex* index2 = NULL;
        if (use_lazzy)
        {
            index1 = new LazyGrid(current_resolution);
            index2 = new LazyGrid(current_resolution);
        }
        else
        {
            index1 = new CellVector();
            index2 = new CellVector();
        }
        std::cout << "before creating static NDTMap" << std::endl;

        NDTMap ndt(index1);
        if (!use_lazzy)
            ndt.loadPointCloud( static_pc, all_indices );
        else
            ndt.loadPointCloud (static_pc );
        ndt.computeNDTCells();
        std::cerr << "static numberOfActiveCells1 : " << ndt.numberOfActiveCells() << std::endl;

        std::cout << "before creating moving NDTMap" << std::endl;

        NDTMap mov(index2);
        if (!use_lazzy)
            mov.loadPointCloud( moving_pc, all_indices_moving );
        else
            mov.loadPointCloud( moving_pc );
        mov.computeNDTCells();

        std::cerr << "static cloud index str : " << ndt.getMyIndexStr() << std::endl;
        std::cerr << "moving cloud index str : " << mov.getMyIndexStr() << std::endl;

        std::cerr << "static numberOfActiveCells : " << ndt.numberOfActiveCells() << std::endl;
        std::cerr << "moving numberOfActiveCells : " << mov.numberOfActiveCells() << std::endl;

        std::cerr << "all_indices.size() : " << all_indices.size() << std::endl;
        std::cerr << "all_indices_moving.size() : " << all_indices_moving.size() << std::endl;
        
        std::cerr << "corresp.size() : " << corresp.size() << std::endl;

        std::cerr << "moving_pc.size() : " << moving_pc.size() << std::endl;

        T_corr = ICPwithCorrMatch(mov, ndt, corresp);
        if (init_T_with_corr) {
            T_f2f = T_corr;
            T_p2p = T_corr;
            T_feat = T_corr;
            T_fusion = T_corr;
            T_d2d = T_corr;
        }

        if (use_f2f)
        {
            matcher.match( mov, ndt, T_f2f );
        }
        if (use_featurecorr)
        {
            std::cout << "use_featurecorr!" << std::endl;
            NDTMatcherFeatureD2D matcher_feat(corresp);
            std::cerr << "MATCHING START" << std::endl;
            matcher_feat.match( mov, ndt, T_feat );
            std::cerr << "MATCHING ENDSTART" << std::endl;
        }
//        delete index;
//    }


        SpatialIndex* index_lazzy1 = new LazyGrid<pcl::PointXYZ>(current_resolution);
        SpatialIndex* index_lazzy2 = new LazyGrid<pcl::PointXYZ>(current_resolution);
        NDTMap lazzy_ndt(index_lazzy1);
        NDTMap lazzy_mov(index_lazzy2);
        
        lazzy_ndt.loadPointCloud( static_pc);
        lazzy_mov.loadPointCloud( moving_pc);
        if (!empty_ndt) {
            lazzy_ndt.computeNDTCells();
            lazzy_mov.computeNDTCells();
        }

        if (use_d2d)
        {
            matcher.match( lazzy_mov, lazzy_ndt, T_d2d );
        }

        if (use_fusion) {
            // This will match using the features with correspondances with normal lazzy NDT grids.
            if (use_lazzy) {
                std::cerr << "Cannot have the lazzy option while doing fusion matching." << std::endl;
                exit(-1);
            }
            std::vector<std::pair<int, int> > corresp2;
            if (!empty_corr)
                corresp2=corresp;
            ROS_ERROR("MATCHING matchFUsion");
            std::cout << "lazzy_ndt.numberOfActiveCells() : " << lazzy_ndt.numberOfActiveCells() << std::endl;
            std::cout << "lazzy_mov.numberOfActiveCells() : " << lazzy_mov.numberOfActiveCells() << std::endl;
            matchFusion(lazzy_mov, lazzy_ndt,
                                                     mov, ndt,
                                                     corresp2,
                                                     T_fusion,
                                                     true,
                                                     use_NDT,
                                                     use_feat,
                                                     step_control);
        }

    std::cout<<"GT translation "<<gt_transform.translation().transpose()
             <<" (norm) "<<gt_transform.translation().norm()<<std::endl;
    std::cout<<"GT rotation "<<gt_transform.rotation().eulerAngles(0,1,2).transpose()
             <<" (norm) "<<gt_transform.rotation().eulerAngles(0,1,2).norm()<<std::endl;

    std::cout<<"T_corr translation "<<T_corr.translation().transpose()
             <<" (norm) "<<T_corr.translation().norm()<<std::endl;
    std::cout<<"T_corr rotation "<<T_corr.rotation().eulerAngles(0,1,2).transpose()
             <<" (norm) "<<T_corr.rotation().eulerAngles(0,1,2).norm()<<std::endl;


    if (use_f2f)
    {
        std::cout<<"f2f translation "<<T_f2f.translation().transpose()
                 <<" (norm) "<<T_f2f.translation().norm()<<std::endl;
        std::cout<<"f2f rotation "<<T_f2f.rotation().eulerAngles(0,1,2).transpose()
                 <<" (norm) "<<T_f2f.rotation().eulerAngles(0,1,2).norm()<<std::endl;
    }

    if (use_featurecorr)
    {
        std::cout<<"feat translation "<<T_feat.translation().transpose()
                 <<" (norm) "<<T_feat.translation().norm()<<std::endl;
        std::cout<<"feat rotation "<<T_feat.rotation().eulerAngles(0,1,2).transpose()
                 <<" (norm) "<<T_feat.rotation().eulerAngles(0,1,2).norm()<<std::endl;
    }
    if (use_fusion)
    {
        std::cout<<"fusion translation "<<T_fusion.translation().transpose()
                 <<" (norm) "<<T_fusion.translation().norm()<<std::endl;
        std::cout<<"fusion rotation "<<T_fusion.rotation().eulerAngles(0,1,2).transpose()
                 <<" (norm) "<<T_fusion.rotation().eulerAngles(0,1,2).norm()<<std::endl;
    }
    if (use_d2d)
    {
        std::cout<<"d2d translation "<<T_d2d.translation().transpose()
                 <<" (norm) "<<T_d2d.translation().norm()<<std::endl;
        std::cout<<"d2d rotation "<<T_d2d.rotation().eulerAngles(0,1,2).transpose()
                 <<" (norm) "<<T_d2d.rotation().eulerAngles(0,1,2).norm()<<std::endl;
    }
    if (use_p2preg)
    {
        std::cout<<"p2preg translation "<<T_p2p.translation().transpose()
                 <<" (norm) "<<T_p2p.translation().norm()<<std::endl;
        std::cout<<"p2preg rotation "<<T_p2p.rotation().eulerAngles(0,1,2).transpose()
                 <<" (norm) "<<T_p2p.rotation().eulerAngles(0,1,2).norm()<<std::endl;
    }
    cout << "done." << endl;

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        marker_pub.publish(ndt_visualisation::markerNDTCells(ndt, 1));
        marker_pub.publish(ndt_visualisation::markerNDTCells(mov, 2));
        marker_pub.publish(ndt_visualisation::markerCellVectorCorrespondances(ndt, mov, corresp));

        marker_pub.publish(ndt_visualisation::markerNDTCells(lazzy_ndt, 3));
        marker_pub.publish(ndt_visualisation::markerNDTCells(lazzy_mov, 4));
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete index1;
    delete index2;
#endif
}
