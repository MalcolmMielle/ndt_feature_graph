#ifndef NDT_GRAPH_DISPLAY_H_30032017
#define NDT_GRAPH_DISPLAY_H_30032017

#include <boost/circular_buffer.hpp>

#include <ndt_map/NDTMapMsg.h>
#include <rviz/message_filter_display.h>
#include "ndt_feature/NDTGraphMsg.h"

namespace Ogre
{
  class SceneNode;
}

namespace rviz
{
  class ColorProperty;
  class FloatProperty;
  class IntProperty;
}

namespace lslgeneric{

  class NDTLineVisual;
  
}

namespace perception_oru{
	namespace ndt_rviz_visualisation{

  class NDTGraphDisplay: public rviz::MessageFilterDisplay<ndt_feature::NDTGraphMsg>{
    Q_OBJECT
    public:

    NDTGraphDisplay();
    virtual ~NDTGraphDisplay();

  protected:
    virtual void onInitialize();

    virtual void reset();

  private Q_SLOTS:
    void updateColorAndAlpha();
    void updateHistoryLength();

  private:
    void processMessage(const ndt_feature::NDTGraphMsg::ConstPtr& msg);

    std::deque<boost::shared_ptr<lslgeneric::NDTLineVisual> > visuals_;

    rviz::ColorProperty* color_property_;
    rviz::FloatProperty* alpha_property_;
    rviz::IntProperty* history_length_property_;
  };
}
}

#endif 

