#ifndef NDTFEATURE_NDTCORNERNNODE_24072016
#define NDTFEATURE_NDTCORNERNODE_24072016

#include "Eigen/Core"

namespace ndt_feature{
	
	class NDTCornerNode {
	
	protected:
		
		Eigen::Vector3d _corners_position;
		std::vector<int> _seen_by;
		
	public:
		NDTCornerNode(){};
		
		void setPose(const Eigen::Vector3d& pos){_corners_position = pos;}
		void seenBy(int i){_seen_by.push_back(i);}
		bool wasSeenBy(int in) const {
			for(size_t i = 0 ; i < _seen_by.size() ; ++i){
				if (in == _seen_by[i]){
					return true;
				}
			}
			return false;
		}
		
		std::vector<int>& getSeen(){return _seen_by;}
		const std::vector<int>& getSeen() const {return _seen_by;}
		Eigen::Vector3d& getPose(){return _corners_position;}
		const Eigen::Vector3d& getPose() const {return _corners_position;}
		void clear(){_seen_by.clear();}
		
		
		
	};
}

#endif