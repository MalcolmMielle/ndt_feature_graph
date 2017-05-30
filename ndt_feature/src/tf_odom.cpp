#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#define TOLERANCE 0.00001f
#define PIOVER180 3.14159/180


geometry_msgs::TransformStamped odom_trans;

	
class Quaternion
{
	protected : 
	float x;
	float y;
	float z;
	float w;
	public : 
	Quaternion(float roll, float pitch, float yaw){
		// Basically we create 3 Quaternions, one for pitch, one for yaw, one for roll
		// and multiply those together.
		// the calculation below does the same, just shorter
		float p = pitch * PIOVER180 / 2.0;
		float y = yaw * PIOVER180 / 2.0;
		float r = roll * PIOVER180 / 2.0;
	 
		float sinp = sin(p);
		float siny = sin(y);
		float sinr = sin(r);
		float cosp = cos(p);
		float cosy = cos(y);
		float cosr = cos(r);
		this->x = sinr * cosp * cosy - cosr * sinp * siny;
		this->y = cosr * sinp * cosy + sinr * cosp * siny;
		this->z = cosr * cosp * siny - sinr * sinp * cosy;
		this->w = cosr * cosp * cosy + sinr * sinp * siny;
		normalise();
	}
	
	Quaternion(float x, float y, float z, float w){
		x=x;
		y=y;
		z=z;
		w=w;
	}
	
	float getX(){return x;}
	float getY(){return y;}
	float getZ(){return z;}
	float getW(){return w;}
	
	
	// normalising a quaternion works similar to a vector. This method will not do anything
	// if the quaternion is close enough to being unit-length. define TOLERANCE as something
	// small like 0.00001f to get accurate results
	void normalise()
	{
		// Don't normalize if we don't have to
		float mag2 = w * w + x * x + y * y + z * z;
		if (fabs(mag2) > TOLERANCE && fabs(mag2 - 1.0f) > TOLERANCE) {
			float mag = sqrt(mag2);
			w /= mag;
			x /= mag;
			y /= mag;
			z /= mag;
		}
	}
	
	
	
	Quaternion getConjugate()
	{
		return Quaternion(-x, -y, -z, w);
	}
/*	
	void getAxisAngle(Eigen::Vector3d& axis, float* angle)
	{
		float scale = sqrt(x * x + y * y + z * z);
		axis(0) = x / scale;
		axis(1) = y / scale;
		axis(2) = z / scale;
		*angle = acos(w) * 2.0f;
	}*/

	
};



void getOdom(const nav_msgs::Odometry::ConstPtr& odomRead, tf::TransformBroadcaster& odom_broadcaster, tf::TransformBroadcaster& worldodom_broadcaster){
	std::cout << "Odom " << odomRead->header.stamp << std::endl;
	
	odom_trans.header.stamp = odomRead->header.stamp;
	odom_trans.header.frame_id = "odom_frame";
	odom_trans.child_frame_id = "base_footprint";

	//send the transform
	odom_broadcaster.sendTransform(odom_trans);
	
	odom_trans.header.frame_id = "world";
	odom_trans.child_frame_id = "odom";
	worldodom_broadcaster.sendTransform(odom_trans);
	
}



// void world(tf::TransformBroadcaster& odom_broadcaster){
// 	std::cout << "Odom " << odomRead->header.stamp << std::endl;
// 	geometry_msgs::TransformStamped odom_trans;
// 	odom_trans.header.stamp = odomRead->header.stamp;
// // 	odom_trans.header.stamp = current_time;
// 	odom_trans.header.frame_id = "odom_frame";
// 	odom_trans.child_frame_id = "base_footprint";
// // 	odom_trans.transform.translation.x = odomRead->pose.pose.position.x;
// // 	odom_trans.transform.translation.y = odomRead->pose.pose.position.y;
// // 	odom_trans.transform.translation.z = odomRead->pose.pose.position.z;
// 	odom_trans.transform.rotation = odomRead->pose.pose.orientation;
// 	odom_trans.transform.translation.x = 0;
// 	odom_trans.transform.translation.y = 0;
// 	odom_trans.transform.translation.z = 0;
// 	Quaternion q(0,0,0);
// 	odom_trans.transform.rotation.x = q.getX();
// 	odom_trans.transform.rotation.y = q.getY();
// 	odom_trans.transform.rotation.z = q.getZ();
// 	odom_trans.transform.rotation.w = q.getW();
// 	//send the transform
// 	odom_broadcaster.sendTransform(odom_trans);
// }



int main(int argc, char** argv){
  
	ros::init(argc, argv, "odometry_publisher");

	tf::TransformBroadcaster odom_broadcaster;
	tf::TransformBroadcaster worldodom_broadcaster;
	ros::NodeHandle n("~");
	ros::Subscriber odom_pub = n.subscribe<nav_msgs::Odometry>("/encoder", 50, boost::bind(&getOdom, _1, odom_broadcaster, worldodom_broadcaster));
//   ros::init();
// 	ros::Rate r(1.0);
	odom_trans.transform.translation.x = 0;
	odom_trans.transform.translation.y = 0;
	odom_trans.transform.translation.z = 0;
	Quaternion q(0,0,0);
	odom_trans.transform.rotation.x = q.getX();
	odom_trans.transform.rotation.y = q.getY();
	odom_trans.transform.rotation.z = q.getZ();
	odom_trans.transform.rotation.w = q.getW();
	
	while(n.ok()){
		ros::spinOnce();               // check for incoming messages
		
		
		
	// 	odom_trans.transform.translation.x = odomRead->pose.pose.position.x;
	// 	odom_trans.transform.translation.y = odomRead->pose.pose.position.y;
	// 	odom_trans.transform.translation.z = odomRead->pose.pose.position.z;

		//send the transform
		
	}
}