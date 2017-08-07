#ifndef NDTFEATURE_MOTIONMODEL_1907201
#define NDTFEATURE_MOTIONMODEL_1907201

#include <angles/angles.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <iostream>

namespace ndt_feature {
	
	typedef Eigen::Vector3d Pose2d;

     //! Holds a pose 2d (x,y,th) with covariance (3x3)
     class Pose2dCov
     {
     public:
	  //! Default contstructor, will initialize zeros everywhere.
	  Pose2dCov()
	  {
	       mean.setZero();
	       cov.setZero();
	  }
	  //! Initialize using the provided parameters.
	  Pose2dCov(double x, double y, double t, double std_xx, double std_xy, double std_xt, double std_yy, double std_yt, double std_tt)
	  {
	       mean[0] = x;
	       mean[1] = y;
	       mean[2] = t;
	       cov(0,0) = std_xx;
	       cov(1,0) = cov(0,1) = std_xy;
	       cov(2,0) = cov(0,2) = std_xt;
	       cov(1,1) = std_yy;
	       cov(1,2) = cov(2,1) = std_yt;
	       cov(2,2) = std_tt;
	  }
	  Pose2d mean;
	  Eigen::Matrix3d cov;
	  friend std::ostream& operator<<(std::ostream &os, const ndt_feature::Pose2dCov &obj)
	  {
	       os << "\n[mean] : \n" << obj.mean;
	       os << "\n[cov ] : \n" << obj.cov;
	       return os;
	  }
     };

     //! Return a Pose2d (a eigen::Vector3) where the incr pose (given in local coordinates) have been added to origin.
     Pose2d addPose2d(const Pose2d &origin, const Pose2d &incr);

     //! Return the relative pose between the origin and the 'pose'.
     Pose2d subPose2d(const Pose2d &origin, const Pose2d &pose);

     //! Return a Pose2dCov (a eigen::Vector3) where the incr pose (given in local coordinates) have been added to origin.
     /*!
      * This corrects the returned covariance by the jacobian.
      */
     Pose2dCov addPose2dCov(const Pose2dCov &origin, const Pose2dCov &incr);

     // Return the Euclidean distance to the origin.
     inline double getDist(const Pose2d &incr)
     {
	  return sqrt(pow(incr[0],2)+pow(incr[1],2));
     }

     inline double getDistBetween(const Pose2d &pose1, const Pose2d &pose2)
     {
       Pose2d tmp = pose1 - pose2;
       return getDist(tmp);
     }

     inline double getAngularNormDist(const Pose2d &pose1, const Pose2d &pose2)
     {
       return angles::normalize_angle(pose1[2] - pose2[2]);
     }

     inline Eigen::Vector2d getPosition(const Pose2d &pose)
     {
       return Eigen::Vector2d(pose(0),pose(1));
     }

     inline double getHeading(const Pose2d &pose)
     {
       return pose[2];
     }

     inline Pose2d getBaseOffsetPose(const Pose2d &pose, double offset)
     {
       Pose2d opose(0,offset,0);
       return addPose2d(pose,opose);
     }

     bool addStepPose2d(const ndt_feature::Pose2d &p1, const ndt_feature::Pose2d &p2, double maxDist, double maxRotation);

     // Return a point which lies along the base (local y-axis)
     Eigen::Vector2d getBaseOffset(const Pose2d &pose, double offset);
     double getDirectionIncr(const Pose2d &p);

     //! Returns the 'direction of motion' between two poses. 1 -> forward, -1 -> reverse.
     double getDirection(const Pose2d &p1, const Pose2d &p2);

     bool forwardDirection(const Pose2d &p1, const Pose2d &p2);

     Eigen::Quaterniond getQuaterion(const Pose2d &p);


     bool validPose2dDiff(const Pose2d &p1, const Pose2d &p2, double maxDist, double maxAngularDist);

     //! Matrix inversion using SVD.
     // TODO - move to another location.
     Eigen::MatrixXd pseudoInverse(Eigen::MatrixXd &mat);

	ndt_feature::Pose2d pose2dFromAffine3d(const Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T);

	Eigen::Matrix3d cov6toCov3(const Eigen::MatrixXd &cov6);

	void pose2dClearDependence(ndt_feature::Pose2dCov &posecov);
	
	
	class MotionModel2d
	{
	public:
	//! Holds params for the motion models.
	class Params
	{
	public:
		//! Constructor, initiate to reasonable params.
		/*! 
		* Same notation/params as in Eliazar, Learning Probabilistic Motion Models for Mobile Robots.
		*/
		Params() 
		{
		Cd = 0.001;
		Ct = 0.001;
		Dd = 0.005;
		Dt = 0.005;
		Td = 0.001;
		Tt = 0.001;
			}
		
		//! Cd -> variance sideways from distance traveled
		double Cd;
		//! Ct -> variance sideways from rotation
		double Ct;
		//! Dd -> variance forward from distance traveled
		double Dd;
		//! Dt -> variance forward from rotation
		double Dt;
		//! Td -> variance in rotation from distance traveled
		double Td;
		//! Tt -> variance in rotation from rotation
		double Tt;
		//! Display the parameters.
		friend std::ostream& operator<<(std::ostream &os, const MotionModel2d::Params &obj)
		{
		os << "\nCd      : " << obj.Cd;
		os << "\nCt      : " << obj.Ct;
		os << "\nDd      : " << obj.Dd;
		os << "\nDt      : " << obj.Dt;
		os << "\nTd      : " << obj.Td;
		os << "\nTt      : " << obj.Tt;
		return os;
		}
	};
	
	MotionModel2d() { }
	MotionModel2d(const MotionModel2d::Params &p) : params(p) { }
	
	void setParams(const MotionModel2d::Params &p)
	{
		params = p;
	}
	//! Obtain the covariance for the provided relative incremental pose
	ndt_feature::Pose2dCov getPose2dCov(const ndt_feature::Pose2d &rel) const;

	Eigen::MatrixXd getCovMatrix6(const ndt_feature::Pose2d &rel) const;

	MotionModel2d::Params params;
	
	private:
	Eigen::Matrix3d getMeasurementCov(const Eigen::Vector3d &relativePose) const;
	
	};

}

// // Forward declaration of class boost::serialization::access
// namespace boost {
//   namespace serialization {
//     class access;
//     
//     // Serialization of Eigen::Vector2d
//     template<typename Archive>
//     inline void serialize(Archive& ar, ndt_feature::MotionModel2d::Params& o, const unsigned int version) {
//       ar & o.Cd
//         & o.Ct
//         & o.Dd
//         & o.Dt
//         & o.Td
//         & o.Tt;
//     }
//   }
// }

#endif