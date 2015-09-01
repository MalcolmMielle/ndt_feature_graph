// Test code to evaluate how laser2d features such as the FLIRT library could paly along with NDT.
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"
#include <cstdio>
#include <Eigen/Eigen>
#include <fstream>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/circular_buffer.hpp>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_srvs/Empty.h>

#include <ndt_feature/ndt_rviz.h>

#include <flirtlib_ros/flirtlib.h>
#include <flirtlib_ros/conversions.h>

#include <ndt_feature/flirtlib_utils.h>
#include <ndt_feature/conversions.h>
#include <ndt_feature/ndt_feature_rviz.h>

class NDTFeature2DViewNode {
    
protected:
    // Our NodeHandle
    ros::NodeHandle nh_;
    
    ros::Subscriber laser_sub_;

    std::string laser_topic, world_frame;
    laser_geometry::LaserProjection projector_;
    ros::Publisher marker_pub_;
    double min_laser_range_;
    double varz, resolution;

    // Flirtlib
    boost::shared_ptr<SimpleMinMaxPeakFinder> peak_finder_;
    boost::shared_ptr<HistogramDistance<double> > histogram_dist_;
    boost::shared_ptr<Detector> detector_;
    boost::shared_ptr<DescriptorGenerator> descriptor_;
    

public:
    // Constructor
    NDTFeature2DViewNode(ros::NodeHandle param_nh) : 
        peak_finder_(ndt_feature::createPeakFinder()),
        histogram_dist_(new SymmetricChi2Distance<double>()),
        detector_(ndt_feature::createDetector(peak_finder_.get())),
        descriptor_(ndt_feature::createDescriptor(histogram_dist_.get()))
    {
        ///topic to wait for laser scan messages, if available
        param_nh.param<std::string>("laser_topic",laser_topic,"laser_scan");
        //the world frame
        param_nh.param<std::string>("world_frame",world_frame,"/world");
        param_nh.param("min_laser_range",min_laser_range_,0.1);
        param_nh.param("resolution",resolution,0.10);
        param_nh.param("laser_variance_z",varz,resolution/4);

        laser_sub_ = nh_.subscribe(laser_topic, 1, &NDTFeature2DViewNode::laserCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    }
    
    ~NDTFeature2DViewNode()
    {
        
    }
    
    // Callback
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
	{
            // Compute the flirt features
            boost::shared_ptr<LaserReading> laser_reading = flirtlib_ros::fromRos(*msg_in);
            for (unsigned int i = 0; i< laser_reading->getRho().size(); i++) {
            }
            InterestPointVec pts;
            detector_->detect(*laser_reading, pts);
            
            // Visualize
            geometry_msgs::Pose pose;
            {
                pose.position.x = 0.;
                pose.position.y = 0.;
                pose.position.z = 0.;
                pose.orientation = tf::createQuaternionMsgFromYaw(0.);
            }
            marker_pub_.publish(flirtlib_ros::interestPointMarkers(pts, pose, 0));
            marker_pub_.publish(ndt_feature::interestPointSupportMarkers(pts, pose, 0));

            // Convert to pcl cloud and indices (to use the exising ndtmap API).
            pcl::PointCloud<pcl::PointXYZ> pcl_ip_cloud;
            std::vector<std::vector<size_t> > indices;
            ndt_feature::convertInterestPointVecToPointClouds(pts, varz, pcl_ip_cloud, indices);
            
            // Create the NDT feature map
            lslgeneric::CellVector* cv = new lslgeneric::CellVector();
            lslgeneric::SpatialIndex* index = cv; //new lslgeneric::CellVector<pcl::PointXYZ>();
            lslgeneric::NDTMap ndt_feat(index);
            // TODO::: get a way to compute the covariance. The support size is not a good option to use directly. One idea is to use a local region around the point e.g. 0.5 meters around and compute a covariance there. This is not at all straight forward todo...
// ndt_feat.loadPointCloud(pcl_ip_cloud, indices);
            //ndt_feat.computeNDTCellsSimple(); // Don't work, the support size is varying to much to be useful here...
            Eigen::Matrix3d cov;
            cov << 0.005, 0., 0., 0., 0.01, 0., 0., 0., 0.01;
            ndt_feature::addInterestPointsToCellVectorFixedCov(pts, cov, cv);
            
            sensor_msgs::PointCloud2 cloud;
	    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_unfiltered, pcl_cloud;
	    projector_.projectLaser(*msg_in, cloud);
	    pcl::fromROSMsg (cloud, pcl_cloud_unfiltered);

	    pcl::PointXYZ pt;
	    //add some variance on z
	    for(int i=0; i<pcl_cloud_unfiltered.points.size(); i++) {
		pt = pcl_cloud_unfiltered.points[i];
		if(sqrt(pt.x*pt.x+pt.y*pt.y) > min_laser_range_) {
		    pt.z += varz*((double)rand())/(double)INT_MAX;
		    pcl_cloud.points.push_back(pt);
		}
	    }

            // Create a NDTMap
            lslgeneric::SpatialIndex* index2 = new lslgeneric::LazyGrid(resolution);
            lslgeneric::NDTMap ndt(index2);
            ndt.loadPointCloud (pcl_cloud);
            ndt.computeNDTCells();
            
            // Visualize
            marker_pub_.publish(ndt_visualisation::markerNDTCells(ndt, 1));
            marker_pub_.publish(ndt_visualisation::markerNDTCells(ndt_feat, 2));

            delete index;
            delete index2;
	};
	

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ndt_feature2d_view");

    ros::NodeHandle param("~");
    NDTFeature2DViewNode t(param);
    ros::spin();

    return 0;
}
