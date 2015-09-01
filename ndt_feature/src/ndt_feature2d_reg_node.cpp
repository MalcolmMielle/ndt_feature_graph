// Test code to evaluate how laser2d features such as the FLIRT library could paly along with NDT.
#include <ndt_fuser/ndt_fuser_hmt.h>
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

#include <ndt_visualisation/ndt_rviz.h>

#include <flirtlib_ros/flirtlib.h>
#include <flirtlib_ros/conversions.h>


class NDTFeature2DViewNode {
    
protected:
    // Our NodeHandle
    ros::NodeHandle nh_;
    
    message_filters::Subscriber<sensor_msgs::LaserScan> *laser_sub_;

    std::string laser_topic, world_frame;
    laser_geometry::LaserProjection projector_;
    ros::Publisher marker_pub_;

    vector<sensor_msg::LaserScan> scans_;

public:
    // Constructor
    NDTFeature2DViewNode(ros::NodeHandle param_nh) : nb_added_clouds_(0)
    {
        ///topic to wait for laser scan messages, if available
        param_nh.param<std::string>("laser_topic",laser_topic,"laser_scan");
        //the world frame
        param_nh.param<std::string>("world_frame",world_frame,"/world");
        ///enable for LaserScan message input
        param_nh.param("matchLaser",matchLaser,false);
	
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    }
    
    ~NDTFeature2DViewNode()
    {
        
    }
    
    // Callback
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
	{
            scans_.push_back(*msg_in);
            
            
            // Add to a queue
	    sensor_msgs::PointCloud2 cloud;
	    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_unfiltered, pcl_cloud;
	    message_m.lock();
	    projector_.projectLaser(*msg_in, cloud);
	    message_m.unlock();
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
	    //ROS_INFO("Got laser points");

	    T.setIdentity();
	    this->processFrame(pcl_cloud,T);

	};
	

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ndt_feature2d_view");

    ros::NodeHandle param("~");
    NDTFuserNode t(param);
    ros::spin();

    return 0;
}

