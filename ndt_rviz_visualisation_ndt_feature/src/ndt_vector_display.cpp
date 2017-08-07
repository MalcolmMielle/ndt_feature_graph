#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "ndt_rviz_visualisation/ndt_line_visual.hpp"
#include "ndt_vector_display.hpp"

namespace perception_oru{
	namespace ndt_rviz_visualisation {
  
  NDTVectorMapDisplay::NDTVectorMapDisplay(){
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
  void NDTVectorMapDisplay::onInitialize(){
    MFDClass::onInitialize();
  }

  NDTVectorMapDisplay::~NDTVectorMapDisplay(){
  }
  
  void NDTVectorMapDisplay::reset(){
    MFDClass::reset();
    visuals_.clear();
// 	all_visuals_.clear();
  }
  
  void NDTVectorMapDisplay::updateColorAndAlpha(){
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
  
void NDTVectorMapDisplay::updateHistoryLength()
{
	ROS_INFO_STREAM("history received: " << this->history_length_property_->getInt());
	 
}

  
  void NDTVectorMapDisplay::processMessage(const ndt_map::NDTVectorMapMsg::ConstPtr& msg ){
    ROS_INFO_STREAM(" vector MESSAGE RECIVED with history: " << this->history_length_property_->getInt() << " and deque size " << visuals_.size() );
	
	std::cout << "MSG detail. nb of maps: " <<msg->maps.size() << std::endl;
	
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
	
	visuals_.clear();
    
    for(int i = 0 ; i < msg->maps.size() ; ++i){
		
		std::cout << "For map " << i << " there is " << msg->maps[i].cells.size() << " cells " << std::endl;
		
		position.x = msg->transformations[i].translation.x;
		position.y = msg->transformations[i].translation.y;
		position.z = msg->transformations[i].translation.z;
		
		orientation.x = msg->transformations[i].rotation.x;
		orientation.y = msg->transformations[i].rotation.y;
		orientation.z = msg->transformations[i].rotation.z;
		orientation.w = msg->transformations[i].rotation.w;
		
		for(int itr=0;itr<msg->maps[i].cells.size(); itr++){
	
			boost::shared_ptr<lslgeneric::NDTLineVisual> visual;
			visual.reset(new lslgeneric::NDTLineVisual(context_->getSceneManager(), scene_node_));
			if(!(msg->maps[i].x_cell_size == msg->maps[i].y_cell_size && msg->maps[i].y_cell_size == msg->maps[i].z_cell_size)){ 
				ROS_ERROR("SOMETHING HAS GONE VERY WRONG YOUR VOXELL IS NOT A CUBE"); 
				//return false;
			}
			
			visual->setCell(msg->maps[i].cells[itr], msg->maps[i].x_cell_size);
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
PLUGINLIB_EXPORT_CLASS(perception_oru::ndt_rviz_visualisation::NDTVectorMapDisplay,rviz::Display)

