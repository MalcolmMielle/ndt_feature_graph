#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "ndt_rviz_visualisation/ndt_line_visual.hpp"
#include "ndt_graph_display.hpp"

namespace perception_oru{
	namespace ndt_rviz_visualisation{
  
  NDTGraphDisplay::NDTGraphDisplay(){
    ROS_ERROR("BUILDING OBJECT");
    color_property_ = new rviz::ColorProperty( "Color", QColor( 204, 51, 204 ),
                                               "Color to draw the acceleration arrows.",
                                               this, SLOT( updateColorAndAlpha() ));

    alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                               "0 is fully transparent, 1.0 is fully opaque.",
                                               this, SLOT( updateColorAndAlpha() ));

    history_length_property_ = new rviz::IntProperty( "History Length", 1,
                                                      "Number of prior measurements to display.",
                                                      this, SLOT( updateHistoryLength() ));
    history_length_property_->setMin( 1 );
    history_length_property_->setMax( 100000 );
  }
  void NDTGraphDisplay::onInitialize(){
    MFDClass::onInitialize();
  }

  NDTGraphDisplay::~NDTGraphDisplay(){
  }
  
  void NDTGraphDisplay::reset(){
    MFDClass::reset();
    visuals_.clear();
// 	all_visuals_.clear();
  }
  
  void NDTGraphDisplay::updateColorAndAlpha(){
    float alpha=alpha_property_->getFloat();
    Ogre::ColourValue color=color_property_->getOgreColor();
    for(size_t i=0;i<visuals_.size();i++){
      visuals_[i]->setColor(color.r,color.g,color.b,alpha);
    }
//     for(size_t i=0;i<all_visuals_.size();i++){
//       for(size_t j=0;j<all_visuals_[i].size();j++){
// 		all_visuals_[i][j]->setColor(color.r,color.g,color.b,alpha);
// 	  }
//     }
  }
  
void NDTGraphDisplay::updateHistoryLength()
{
	ROS_INFO_STREAM("history received: " << this->history_length_property_->getInt());
}

  
  void NDTGraphDisplay::processMessage(const ndt_feature::NDTGraphMsg::ConstPtr& msg ){
    ROS_INFO_STREAM("graph MESSAGE RECIVED with history: " << this->history_length_property_->getInt() << " and deque size " << visuals_.size() );
	
	std::cout << "MSG detail. nb of maps: " <<msg->nodes.size() << std::endl;
	
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
	
	visuals_.clear();
    
    for(int i = 0 ; i < msg->nodes.size() ; ++i){
		
		std::cout << "For map " << i << " there is " << msg->nodes[i].map.map.cells.size() << " cells " << std::endl;

		position.x = msg->nodes[i].T.position.x;
		position.y = msg->nodes[i].T.position.y;
		position.z = msg->nodes[i].T.position.z;
		
		orientation.x = msg->nodes[i].T.orientation.x;
		orientation.y = msg->nodes[i].T.orientation.y;
		orientation.z = msg->nodes[i].T.orientation.z;
		orientation.w = msg->nodes[i].T.orientation.w;
		
		for(int itr=0;itr<msg->nodes[i].map.map.cells.size();itr++){
			
			boost::shared_ptr<lslgeneric::NDTLineVisual> visual;
			visual.reset(new lslgeneric::NDTLineVisual(context_->getSceneManager(), scene_node_));
			if(!(msg->nodes[i].map.map.x_cell_size==msg->nodes[i].map.map.y_cell_size&&msg->nodes[i].map.map.y_cell_size==msg->nodes[i].map.map.z_cell_size)){ 
				ROS_ERROR("SOMETHING HAS GONE VERY WRONG YOUR VOXELL IS NOT A CUBE"); 
				//return false;
			}
			
			visual->setCell(msg->nodes[i].map.map.cells[itr],msg->nodes[i].map.map.x_cell_size);
			visual->setFramePosition(position);
			visual->setFrameOrientation(orientation);
			float alpha = alpha_property_->getFloat();
			Ogre::ColourValue color=color_property_->getOgreColor();
			visual->setColor(color.r,color.g,color.b,alpha);
			visuals_.push_back(visual);
				
		}
	}
    
    
  }
}
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(perception_oru::ndt_rviz_visualisation::NDTGraphDisplay,rviz::Display)

