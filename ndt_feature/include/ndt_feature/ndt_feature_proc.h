#ifndef NDT_FEATURE_FRAME_PROC_HH
#define NDT_FEATURE_FRAME_PROC_HH


namespace ndt_feature_frame {

  inline bool match(NDTFeatureFrame &target,
	     NDTFeatureFrame &source,
	     Eigen::Affine3d &T,
	     bool useInitialGuess) {
        

  }

  inline bool fuse(NDTFeatureFrame &target,
	    NDTFeatureFrame &source,
	    Eigen::Affine3d &T);


} // namespace

#endif
