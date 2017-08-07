#include "ndt_feature/motion_model.hpp"
	
	
	
ndt_feature::Pose2d ndt_feature::addPose2d(const ndt_feature::Pose2d& origin, const ndt_feature::Pose2d& incr){
	Eigen::Vector3d ret;
	ret[0] = origin[0] + incr[0]*cos(origin[2]) - incr[1]*sin(origin[2]);
	ret[1] = origin[1] + incr[0]*sin(origin[2]) + incr[1]*cos(origin[2]);
	ret[2] = angles::normalize_angle(origin[2] + incr[2]);
	return ret;
}

     //! Return the relative pose between the origin and the 'pose'.
ndt_feature::Pose2d ndt_feature::subPose2d(const Pose2d &origin, const Pose2d &pose)
{
	Eigen::Vector3d ret;
	double cos_ = cos(origin[2]);
	double sin_ = sin(origin[2]);
	ret[0] =  (pose[0] - origin[0]) * cos_ + (pose[1] - origin[1]) * sin_;
	ret[1] = -(pose[0] - origin[0]) * sin_ + (pose[1] - origin[1]) * cos_;
	ret[2] = angles::normalize_angle(pose[2] - origin[2]);
	return ret;
}

     //! Return a Pose2dCov (a eigen::Vector3) where the incr pose (given in local coordinates) have been added to origin.
     /*!
      * This corrects the returned covariance by the jacobian.
      */
ndt_feature::Pose2dCov ndt_feature::addPose2dCov(const Pose2dCov &origin, const Pose2dCov &incr)
{
	ndt_feature::Pose2dCov ret;
	ret.mean = addPose2d(origin.mean, incr.mean);

	// Create the Jacobians (this can be found, for example, in Eq. 2.11 in Udo Frese's thesis).
	Eigen::Matrix3d J_1;
	Eigen::Matrix3d J_2;
	double c = cos( origin.mean[2]);
	double s = sin( origin.mean[2]);
	J_1 << 1, 0, -s * incr.mean[0] - c * incr.mean[1],
		0, 1, c * incr.mean[0] - s * incr.mean[1],
		0, 0, 1;

	J_2 << c, -s, 0,
		s, c, 0, 
		0, 0, 1;

	ret.cov = J_1 * origin.cov * J_1.transpose() + 
		J_2 * incr.cov * J_2.transpose();
	return ret;
}

     bool ndt_feature::addStepPose2d(const ndt_feature::Pose2d &p1, const ndt_feature::Pose2d &p2, double maxDist, double maxRotation) {
       if (getDistBetween(p1, p2) > maxDist)
	 return true;
       if (fabs(getAngularNormDist(p1, p2)) > maxRotation)
	 return true;
       return false;
     }

     // Return a point which lies along the base (local y-axis)
     Eigen::Vector2d ndt_feature::getBaseOffset(const Pose2d &pose, double offset)
     {
       return getPosition(getBaseOffsetPose(pose, offset));
     }

     double ndt_feature::getDirectionIncr(const Pose2d &p)
     {
       if (p(0) >= 0) // X is forward.
	 return 1.;
       return -1;
     }

     //! Returns the 'direction of motion' between two poses. 1 -> forward, -1 -> reverse.
     double ndt_feature::getDirection(const Pose2d &p1, const Pose2d &p2)
     {
       Pose2d dir = subPose2d(p1, p2);
       return getDirectionIncr(dir);
     }

     bool ndt_feature::forwardDirection(const Pose2d &p1, const Pose2d &p2)
     {
       if (getDirection(p1,p2) > 0)
	 return true;
       return false;
     }

     Eigen::Quaterniond ndt_feature::getQuaterion(const Pose2d &p) {
     	 double sy = sin(p[2]*0.5);
	 double cy = cos(p[2]*0.5);
	 double sp = 0.;
	 double cp = 1.;
	 double sr = 0.;
	 double cr = 1.;
	 double w = cr*cp*cy + sr*sp*sy;
	 double x = sr*cp*cy - cr*sp*sy;
	 double y = cr*sp*cy + sr*cp*sy;
	 double z = cr*cp*sy - sr*sp*cy;
	 return Eigen::Quaterniond(w,x,y,z);
     }


     bool ndt_feature::validPose2dDiff(const Pose2d &p1, const Pose2d &p2, double maxDist, double maxAngularDist) {
       if (getDistBetween(p1, p2) > maxDist) {
	 std::cout << "distance : " << getDistBetween(p1, p2) << " > " << maxDist << std::endl;
	 return false;
       }
       if (fabs(getAngularNormDist(p1, p2)) > maxAngularDist) {
	 std::cout << "angular distance : " << fabs(getAngularNormDist(p1,p2)) << " > " << maxAngularDist << std::endl;
	 return false;
       }
       return true;
     }

     //! Matrix inversion using SVD.
     // TODO - move to another location.
     Eigen::MatrixXd ndt_feature::pseudoInverse(Eigen::MatrixXd &mat)
     {
	  // Calculate using the SVD.
	  Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
	  //svd.matrixU();
	  //svd.matrixV();
	  Eigen::MatrixXd tmp = svd.singularValues();
	  
	  int size = mat.rows();
	  if (size > mat.cols())
	       size = mat.cols();
	  for (int i = 0; i < size; i++)
	       if (tmp(i,i) < std::numeric_limits<double>::epsilon())
		    tmp(i,i) = 0.;
	       else
		    tmp(i,i) = 1./tmp(i,i);
	  return svd.matrixV().transpose() * tmp.transpose() * svd.matrixU().transpose();
     }

	ndt_feature::Pose2d ndt_feature::pose2dFromAffine3d(const Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T) {
	Pose2d ret;
	ret[0] = T.translation()[0];
	ret[1] = T.translation()[1];
	ret[2] = T.rotation().eulerAngles(0,1,2)[2];
	return ret;
	}

	Eigen::Matrix3d ndt_feature::cov6toCov3(const Eigen::MatrixXd &cov6) {
	Eigen::Matrix3d cov3;
	cov3(0,0) = cov6(0,0);
	cov3(1,1) = cov6(1,1);
	cov3(0,1) = cov6(0,1);
	cov3(1,0) = cov6(1,0);
	cov3(2,2) = cov6(5,5);
	cov3(0,2) = cov6(0,5);
	cov3(1,2) = cov6(1,5);
	cov3(2,0) = cov6(5,0);
	cov3(2,1) = cov6(5,1);
	return cov3;
	}

	void ndt_feature::pose2dClearDependence(ndt_feature::Pose2dCov &posecov) {
	posecov.cov(0,1) = 0.;
	posecov.cov(1,0) = 0.;
	posecov.cov(0,2) = 0.;
	posecov.cov(1,2) = 0.;
	posecov.cov(2,0) = 0.;
	posecov.cov(2,1) = 0.;
	}
	
	
ndt_feature::Pose2dCov ndt_feature::MotionModel2d::getPose2dCov(const ndt_feature::Pose2d& rel) const
{
	Pose2dCov ret;
	ret.mean = rel;
	ret.cov = getMeasurementCov(rel);
	return ret;
}

Eigen::MatrixXd ndt_feature::MotionModel2d::getCovMatrix6(const ndt_feature::Pose2d &rel) const
{
	Eigen::MatrixXd cov(6,6);
	cov.setIdentity();
	Eigen::Matrix3d cov2d = getMeasurementCov(rel);
	
	cov(0,0) = cov2d(0,0);
	cov(1,0) = cov2d(1,0);
	cov(0,1) = cov2d(0,1);
	cov(1,1) = cov2d(1,1);

	cov(0,5) = cov2d(0,2);
	cov(1,5) = cov2d(1,2);

	cov(5,0) = cov2d(2,0);
	cov(5,1) = cov2d(2,1);
	cov(5,5) = cov2d(2,2);

	return cov;
}


Eigen::Matrix3d ndt_feature::MotionModel2d::getMeasurementCov(const Eigen::Vector3d &relativePose) const
{
	double dist = getDist(relativePose);
	double rot = relativePose[2];
	Eigen::Matrix3d R;
	R.setZero();
	R(0,0) = params.Dd*dist*dist + params.Dt*rot*rot;
	R(1,1) = params.Cd*dist*dist + params.Ct*rot*rot;
	R(2,2) = params.Td*dist*dist + params.Tt*rot*rot;
	return R;
}
