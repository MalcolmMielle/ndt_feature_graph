#ifndef NDTFEATURE_VERTEXSE2PRIOR_30082016
#define NDTFEATURE_VERTEXSE2PRIOR_30082016

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"
// #include "types_tutorial_slam2d.h"


#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

// #include "ndt_feature/ndt_feature_graph.h"
// #include "ndt_feature/utils.h"

#include "Eigen/Core"

#include "bettergraph/PseudoGraph.hpp"
#include "vodigrex/linefollower/SimpleNode.hpp"

#include "das/AssociationInterface.hpp"
#include "covariance.hpp"

namespace ndt_feature {
	
	class VertexSE2Prior : public g2o::VertexSE2{
		
		#ifdef G2O_HAVE_OPENGL
  /**
   * \brief visualize the 3D pose vertex
   */
  class G2O_TYPES_SLAM3D_API VertexSE3PriorDrawAction: public DrawAction{
    public:
      VertexSE3DrawAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_);
    protected:
      virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
      FloatProperty* _triangleX, *_triangleY;
  };
#endif
		
	};
	
}

#endif