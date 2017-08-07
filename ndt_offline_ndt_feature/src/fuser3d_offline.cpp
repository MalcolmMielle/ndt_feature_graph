#include <ndt_fuser/ndt_fuser_hmt.h>
#include <ndt_offline/VelodyneBagReader.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_cell.h>
#include <ndt_map/pointcloud_utils.h>
#include <tf_conversions/tf_eigen.h>
#include <cstdio>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <algorithm>

#include <boost/program_options.hpp>
namespace po = boost::program_options;
using namespace std;

inline void normalizeEulerAngles(Eigen::Vector3d &euler) {
  if (fabs(euler[0]) > M_PI/2) {
    euler[0] += M_PI;
    euler[1] += M_PI;
    euler[2] += M_PI;
    
    euler[0] = angles::normalize_angle(euler[0]);
    euler[1] = angles::normalize_angle(euler[1]);
    euler[2] = angles::normalize_angle(euler[2]);
  }
}

template<class T> std::string toString (const T& x)
{
     std::ostringstream o;

     if (!(o << x))
	  throw std::runtime_error ("::toString()");

     return o.str ();
}

void filter_fov_fun(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &cloud_nofilter, double hori_min, double hori_max) {
    for(int i=0; i<cloud_nofilter.points.size(); ++i) {
	double ang = atan2(cloud_nofilter.points[i].y, cloud_nofilter.points[i].x);
	if(ang < hori_min || ang > hori_max) continue;
	cloud.points.push_back(cloud_nofilter.points[i]);
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    std::cout << "nb clouds : " << cloud.points.size() << std::endl;
}


std::string transformToEvalString(const Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T) {
  std::ostringstream stream;
  stream << std::setprecision(std::numeric_limits<double>::digits10);
  Eigen::Quaternion<double> tmp(T.rotation());
  stream << T.translation().transpose() << " " << tmp.x() << " " << tmp.y() << " " << tmp.z() << " " << tmp.w() << std::endl; 
  return stream.str();
}

void saveCloud(int counter, const pcl::PointCloud<pcl::PointXYZ> &cloud) {
    std::string pcd_file = std::string("cloud") + toString(counter) + std::string(".pcd");
    std::cout << "saving : " << pcd_file << std::endl;
    pcl::io::savePCDFileASCII (pcd_file, cloud);
}


/////////////////////////////////////////////////////////////////////////////////7
/////////////////////////////////////////////////////////////////////////////////7
/// *!!MAIN!!*
/////////////////////////////////////////////////////////////////////////////////7
/////////////////////////////////////////////////////////////////////////////////7
int main(int argc, char **argv){
    std::string dirname;
    std::string map_dirname;
    std::string base_name;
    double size_xy;
    double size_z;
    double resolution;
    double resolution_local_factor;
    double sensor_cutoff;
    double hori_min, hori_max;
    double min_dist, min_rot_in_deg;
    double max_range, min_range;
    int itrs;
    int nb_neighbours;
    int nb_scan_msgs;
    lslgeneric::MotionModel2d::Params motion_params;
    std::string tf_base_link, tf_gt_link, tf_world_frame;
    std::string velodyne_config_file;
    std::string velodyne_packets_topic;
    std::string velodyne_frame_id;
    std::string tf_topic;
    Eigen::Vector3d transl;
    Eigen::Vector3d euler;
    double sensor_time_offset;

    po::options_description desc("Allowed options");
    desc.add_options()
	("help", "produce help message")
	("visualize", "visualize the output")
	("use-odometry", "use initial guess from odometry")
	("match2d", "use 2d-matcher")
	("no-step-control", "use step control in the optimization (default=false)")
	("nosubmaps", "run the standard fuser")
	("pre-load", "loads maps from the map directory if available")
	("base-name", po::value<string>(&base_name), "prefix for all generated files")
	("dir-name", po::value<string>(&dirname), "where to look for SCANs")
	("map-dir-name", po::value<string>(&map_dirname), "where to save the pieces of the map (default it ./map)")
	("size-xy", po::value<double>(&size_xy)->default_value(50.), "size of the central map xy")
        ("size-z", po::value<double>(&size_z)->default_value(10.), "height of the map")
        ("resolution", po::value<double>(&resolution)->default_value(1.), "resolution of the map")
        ("resolution_local_factor", po::value<double>(&resolution_local_factor)->default_value(1.), "resolution factor of the local map used in the match and fusing step")
	("sensor_cutoff", po::value<double>(&sensor_cutoff)->default_value(70), "ignore ranges longer than this value")
	("itrs", po::value<int>(&itrs)->default_value(30), "resolution of the map")
	("baseline", "run also the baseline registration algorithms")
	("guess-zpitch", "guess also z and pitch from odometry")
	("use-multires", "run the multi-resolution guess")
	("fuse-incomplete", "fuse in registration estimate even if iterations ran out. may be useful in combination with low itr numbers")
	("filter-fov", "cutoff part of the field of view")
	("hori-max", po::value<double>(&hori_max)->default_value(2*M_PI), "the maximum field of view angle horizontal")
	("hori-min", po::value<double>(&hori_min)->default_value(-hori_max), "the minimum field of view angle horizontal")
        ("COOP", "if parameters from the COOP data set should be used (sensorpose)")
        ("VCE", "if sensorpose parameters from VCE should be used") 
        ("VCEnov16", "if sensorpose parameters from VCE 16 nov data collection should be used")
        ("dustcart", "if the sensorpose parameters from dustcart should be used")
        ("do-soft-constraints", "if soft constraints from odometry should be used")
        ("Dd", po::value<double>(&motion_params.Dd)->default_value(1.), "forward uncertainty on distance traveled")
        ("Dt", po::value<double>(&motion_params.Dt)->default_value(1.), "forward uncertainty on rotation")
        ("Cd", po::value<double>(&motion_params.Cd)->default_value(1.), "side uncertainty on distance traveled")
        ("Ct", po::value<double>(&motion_params.Ct)->default_value(1.), "side uncertainty on rotation")
        ("Td", po::value<double>(&motion_params.Td)->default_value(1.), "rotation uncertainty on distance traveled")
        ("Tt", po::value<double>(&motion_params.Tt)->default_value(1.), "rotation uncertainty on rotation")
        ("min_dist", po::value<double>(&min_dist)->default_value(0.2), "minimum distance traveled before adding cloud")
        ("min_rot_in_deg", po::value<double>(&min_rot_in_deg)->default_value(5), "minimum rotation before adding cloud")
      ("tf_base_link", po::value<std::string>(&tf_base_link)->default_value(std::string("/odom_base_link")), "tf_base_link")
      ("tf_gt_link", po::value<std::string>(&tf_gt_link)->default_value(std::string("/state_base_link")), "tf ground truth link")
      ("velodyne_config_file", po::value<std::string>(&velodyne_config_file)->default_value(std::string("velo32.yaml")), "configuration file for the scanner")
      ("tf_world_frame", po::value<std::string>(&tf_world_frame)->default_value(std::string("/world")), "tf world frame")
      ("velodyne_packets_topic", po::value<std::string>(&velodyne_packets_topic)->default_value(std::string("velodyne_packets")), "velodyne packets topic used")
      ("velodyne_frame_id", po::value<std::string>(&velodyne_frame_id)->default_value(std::string("/velodyne")), "frame_id of the velodyne")
      ("alive", "keep the mapper/visualization running even though it is completed (e.g. to take screen shots etc.")
      ("nb_neighbours", po::value<int>(&nb_neighbours)->default_value(2), "number of neighbours used in the registration")
      ("max_range", po::value<double>(&max_range)->default_value(70.), "maximum range used from scanner")
      ("min_range", po::value<double>(&min_range)->default_value(1.5), "minimum range used from scanner")
      ("save_map", "saves the map at the end of execution")
      ("disable_reg", "disables the registration completetly")
      ("nb_scan_msgs", po::value<int>(&nb_scan_msgs)->default_value(1), "number of scan messages that should be loaded at once from the bag")
      ("use_gt_as_interp_link", "use gt when performing point interplation while unwrapping the velodyne scans")
      ("save_clouds", "save all clouds that are added to the map")
      ("tf_topic", po::value<std::string>(&tf_topic)->default_value(std::string("/tf")), "tf topic to listen to")
       	("x", po::value<double>(&transl[0])->default_value(0.), "sensor pose - translation vector x")
	("y", po::value<double>(&transl[1])->default_value(0.), "sensor pose - translation vector y")
	("z", po::value<double>(&transl[2])->default_value(0.), "sensor pose - translation vector z")
	("ex", po::value<double>(&euler[0])->default_value(0.), "sensor pose - euler angle vector x")
	("ey", po::value<double>(&euler[1])->default_value(0.), "sensor pose - euler angle vector y")
	("ez", po::value<double>(&euler[2])->default_value(0.), "sensor pose - euler angle vector z")
	("sensor_time_offset", po::value<double>(&sensor_time_offset)->default_value(0.), "timeoffset of the scanner data")
	;


    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (!vm.count("base-name") || !vm.count("dir-name"))
    {
	cout << "Missing base or dir names.\n";
	cout << desc << "\n";
	return 1;
    }
    if (vm.count("help"))
    {
	cout << desc << "\n";
	return 1;
    }
    if (!vm.count("map-dir-name"))
    {
	map_dirname="map";
    }
    bool use_odometry = vm.count("use-odometry");
    bool match2d = vm.count("match2d");
    bool visualize = vm.count("visualize");
    bool do_baseline = vm.count("baseline");
    bool guess_zpitch = vm.count("guess-zpitch");
    bool use_multires = vm.count("use-multires");
    bool beHMT = vm.count("nosubmaps")==0;
    bool fuse_incomplete = vm.count("fuse-incomplete");
    bool preload = vm.count("pre-load");
    bool filter_fov = vm.count("filter-fov");
    bool step_control = (vm.count("no-step-control") == 0);
    bool do_soft_constraints = vm.count("do-soft-constraints");
    bool COOP = vm.count("COOP");
    bool VCE = vm.count("VCE");
    bool VCEnov16 = vm.count("VCEnov16");
    bool dustcart = vm.count("dustcart");
    bool alive = vm.count("alive");
    bool save_map = vm.count("save_map");
    bool disable_reg = vm.count("disable_reg");
    bool use_gt_as_interp_link = vm.count("use_gt_as_interp_link");
    bool save_clouds = vm.count("save_clouds");
    
    if(filter_fov) {
	cout << "filtering FOV of sensor to min/max "<<hori_min<<" "<<hori_max<<endl;
    }

    base_name += motion_params.getDescString() + std::string("_res") + toString(resolution) + std::string("_SC") + toString(do_soft_constraints) + std::string("_mindist") + toString(min_dist) + std::string("_sensorcutoff") + toString(sensor_cutoff) + std::string("_stepcontrol") + toString(step_control) + std::string("_neighbours") + toString(nb_neighbours) + std::string("_rlf") + toString(resolution_local_factor);
	
	std::cout << resolution << " " << size_xy << " " << size_xy << " " << size_z << " " << 
                                       sensor_cutoff << " " << visualize << " " << match2d << " " << use_multires << " " << 
                                       fuse_incomplete << " " << itrs << " " << base_name  << " " <<  beHMT  << " " <<  map_dirname << " " << step_control << " " << do_soft_constraints << " " << nb_neighbours << " " << resolution_local_factor << std::endl;

    lslgeneric::NDTFuserHMT ndtslammer(resolution, size_xy, size_xy, size_z, 
                                       sensor_cutoff, visualize, match2d, use_multires, 
                                       fuse_incomplete, itrs, base_name,beHMT,map_dirname,step_control, do_soft_constraints, nb_neighbours, resolution_local_factor);
    ros::Time::init();
    srand(time(NULL));
    bool noOdometry = false;
	
    ndtslammer.disableRegistration = disable_reg;

    /// Set up the sensor link
    tf::StampedTransform sensor_link; ///Link from /odom_base_link -> velodyne
    sensor_link.child_frame_id_ = velodyne_frame_id;
    sensor_link.frame_id_ = tf_base_link; //"/odom_base_link";
    tf::Quaternion quat; 

    quat.setRPY(euler[0], euler[1], euler[2]);
    tf::Vector3 trans(transl[0], transl[1], transl[2]);
    if (COOP) {
      trans[0] = 0.96;
      trans[1] = 0.34;
      trans[2] = 4.0;
      quat.setRPY(0,0,7.2*M_PI/180.0);
    }
    if (VCE) {
      trans[0] = 3.0;
      trans[1] = 0.;
      trans[2] = 3.2; // 1.2
      quat.setRPY(0, 0.18, 0);
    }
    if (VCEnov16) {
	sensor_time_offset = -0.04886; //-0.0921114;
	trans[0] = 3.34925;
        trans[1] = 0.00668733;
	trans[2] = 1.17057819451374;
	quat.setRPY(0.0003905, 0.0331966, 0.0105067);

      // trans[0] = 3.0;//3.0;
      // trans[1] = 0.;
      // trans[2] = 1.2;//1.2;
      // quat.setRPY(0, 0.0, 0); // 0.01
    }
    if (dustcart) {
      trans[0] = -0.5;
      trans[1] = 0.;
      trans[2] = 1.9; // 1.2
      quat.setRPY(0, 0, -1.57);
    }

    tf::Transform T = tf::Transform(quat,trans);
    sensor_link.setData(T);

    std::vector<std::string> scanfiles;
    //list directory in dir_name
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (dirname.c_str())) != NULL) {
	while ((ent = readdir (dir)) != NULL) {
	    if(ent->d_name[0] == '.') continue;
	    char tmpcname[400];
	    snprintf(tmpcname,399,"%s/%s",dirname.c_str(),ent->d_name);
	    std::string tmpfname = tmpcname;
	    scanfiles.push_back(tmpfname);
	}
	closedir (dir);
    } else {
       std::cerr<<"Could not parse dir name\n";
       return -1;	
    }
    sort(scanfiles.begin(),scanfiles.end());
    {
      std::cout << "files to be loaded : " << std::endl;
      for (size_t i = 0; i < scanfiles.size(); i++) {
        std::cout << " " << scanfiles[i] << std::flush;
      }
      std::cout << std::endl;
    }
    Eigen::Affine3d Told,Tprev,Tsold;
    int counter = 0;

    std::ofstream gt_file, odom_file, est_file, sensorpose_est_file;
    std::string filename;
    { 
      filename = base_name + std::string("_gt.txt");
      gt_file.open(filename.c_str());
    }
    { 
      filename = base_name + std::string("_est.txt");
      est_file.open(filename.c_str());
    }
    {
      filename = base_name + std::string("_sensorpose_est.txt");
      sensorpose_est_file.open(filename.c_str());
    }
    { 
      filename = base_name + std::string("_odom.txt");
      odom_file.open(filename.c_str());
    }
    if (!gt_file.is_open() || !est_file.is_open() || !odom_file.is_open())
    {
      ROS_ERROR_STREAM("Failed to open : " << gt_file << " || " << est_file << " || " << odom_file); 
    }


    std::string tf_interp_link = tf_base_link;
    if (use_gt_as_interp_link) {
      tf_interp_link = tf_gt_link;
    }

    for(int i=0; i<scanfiles.size(); i++) {
	std::string bagfilename = scanfiles[i];
	fprintf(stderr,"Opening %s\n",bagfilename.c_str());
	VelodyneBagReader<pcl::PointXYZ> vreader(velodyne_config_file, 
						 bagfilename,
						 velodyne_packets_topic,  //"/velodyne_packets"
						 velodyne_frame_id, 
						 tf_world_frame,
						 tf_topic,
						 ros::Duration(3600),
                                                 &sensor_link, max_range, min_range,
						 sensor_time_offset);  

	pcl::PointCloud<pcl::PointXYZ> cloud, cloud_nofilter;	
	tf::Transform sensor_pose;
	bool cameraset = false;
	int numclouds = 0;
	tf::Transform basepose;
		


	while(vreader.readMultipleMeasurements(nb_scan_msgs,cloud_nofilter,sensor_pose,basepose,/*tf_base_link*//*tf_gt_link*/tf_interp_link/*std::string("/state_base_link")*/)){
	    //if(cloud.size()==0) continue;

		std::cout << "Reading " << std::endl;
	    if(cloud_nofilter.size()==0) continue;
	    tf::Transform baseodo;

	    if(filter_fov) {
		filter_fov_fun(cloud,cloud_nofilter,hori_min,hori_max);
	    } else {
		cloud = cloud_nofilter;
	    }

	    vreader.getPoseFor(baseodo, tf_base_link/*"/odom_base_link"*/);

	    //callback(cloud, sensor_pose,T,basepose, base_odo);	

	    Eigen::Affine3d Todo;
	    Eigen::Affine3d Ttot,Ts,Tbase,Tgt;
#if ROS_VERSION_MINIMUM(1,9,0)
	    tf::transformTFToEigen (sensor_pose, Ttot); ///< This is the velodyne pose 
	    tf::transformTFToEigen (T, Ts); ///<sensor offset
	    tf::transformTFToEigen (basepose, Tgt); ///<Ground truth
	    tf::transformTFToEigen (baseodo, Tbase); ///<Odometry in vehicle frame
#else	
	    tf::TransformTFToEigen (sensor_pose, Ttot); ///< This is the velodyne pose 
	    tf::TransformTFToEigen (T, Ts); ///<sensor offset
	    tf::TransformTFToEigen (basepose, Tgt); ///<Ground truth
	    tf::TransformTFToEigen (baseodo, Tbase); ///<Odometry in vehicle frame
#endif	
	    if(counter == 0){
		counter ++;
		cloud.clear();	
		cloud_nofilter.clear();	
		continue;
	    }
	    if(counter == 1){
                // Make the initialization of the pose to be at the sensory level...
                // Tgt.translation()[2] = 4.;//trans[2];
		Todo=Tgt;
		Tsold = Ttot;
		Tprev = Tgt;

		Eigen::Affine3d Tmotion;
		Tmotion.setIdentity();

		ndtslammer.setSensorPose(Ts);
                ndtslammer.setMotionParams(motion_params);
                
                std::cout << "initializing ..." << std::endl;
		ndtslammer.initialize(Tgt,cloud,preload);
                std::cout << "initializing done" << std::endl;
		Told = Tbase; 
		//Told = Tgt;//for gt

                if (save_clouds) {
		    saveCloud(counter-1, cloud);
                }

		counter ++;
		cloud.clear();	
		cloud_nofilter.clear();	
		continue;
	    }


	    Eigen::Affine3d Tmotion = Told.inverse()*Tbase;
	    Eigen::Vector3d Tmotion_euler = Tmotion.rotation().eulerAngles(0,1,2);
	    normalizeEulerAngles(Tmotion_euler);
	    if(!use_odometry) {
		Tmotion.setIdentity();
	    } else {

  		if(Tmotion.translation().norm()<min_dist && fabs(Tmotion_euler[2])<(min_rot_in_deg*M_PI/180.0)) {
		    cloud.clear();	
		    cloud_nofilter.clear();	
		    continue;
		}
	    }
            if (save_clouds) {
		saveCloud(counter-1, cloud);
            }

	    counter++;
	    Todo = ndtslammer.update(Tmotion,cloud);

	    Tprev = Todo;
	    Told = Tbase;
	    
	    cloud.clear();	
	    cloud_nofilter.clear();	
	    numclouds++;

            // Evaluation 
            ros::Time frame_time = vreader.getTimeStampOfLastSensorMsg();
            gt_file << frame_time << " " << transformToEvalString(Tgt);
            odom_file << frame_time << " " << transformToEvalString(Tbase);
            est_file << frame_time << " " << transformToEvalString(Todo);
            sensorpose_est_file << frame_time << " " << transformToEvalString(Todo * Ts); 
	}
    }

    gt_file.close();
    odom_file.close();
    est_file.close();
    sensorpose_est_file.close();

    if (save_map) {
      std::cout << "Saving map" << std::endl;
      if (ndtslammer.wasInit() && ndtslammer.map != NULL) {
        ndtslammer.map->writeToJFF("map.jff");
        std::cout << "Done." << std::endl;
      }
      else {
        std::cout << "Failed to save map, ndtslammer was not initiated(!)" << std::endl;
      }
    }

    if (alive) {
      while (1) {
        usleep(1000);
      }
    }

    usleep(1000*1000);
    std::cout << "Done." << std::endl;
    // fclose(gtf);
    // fclose(fuserf);
    // fclose(frame2);
    // fclose(fgicp);
    // char c;
    // std::cin >> c;
}
