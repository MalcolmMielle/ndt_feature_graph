#ifndef _TF_MESSAGE_READER_
#define _TF_MESSAGE_READER_
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <tf/message_filter.h>
#include <string>
#include <vector>
#include <pcl/conversions.h>

template<typename MessageType>
class tfMessageReader
{
	public:
		/**
		* Open @bagfilename, subscribe to topic @message_topic_ use tf target_linkname_, which is fixed to fixed_link_
		**/
		tfMessageReader(std::string bagfilename, std::string message_topic_, std::string fixed_link_, std::string target_linkname_){
			message_topic = message_topic_;
			fixed_link = fixed_link_;
			target_linkname = target_linkname_;
			
			fprintf(stderr,"Opening '%s'\n",bagfilename.c_str());
			bag.open(bagfilename, rosbag::bagmode::Read);
			
			std::vector<std::string> topics;
			topics.push_back("/tf");
			topics.push_back(message_topic); 
			
			view = new rosbag::View(bag); //, rosbag::TopicQuery(topics));
			I = view->begin();
			Itf = view->begin();
		}
		
		bool bagEnd(){
			if(I == view->end()) return true;
			return false;
		}
		
		/**
 		* Get tf::Transform by name and timestamp
		*/
		bool getTf(const std::string &frame_name,const ros::Time &stamp, tf::Transform &pose){
			std::string schaiba;
			schaiba.resize(500);
			if(!transformer.canTransform(fixed_link, frame_name, stamp, &schaiba)){
				fprintf(stderr,"WTF:%s\n",schaiba.c_str());
				return false;
			}
			
			tf::StampedTransform transform;
			transformer.lookupTransform(fixed_link, frame_name, stamp,transform);
			pose = transform;
			return true;
		}
		
		
		/**
		* Read next message
		* Returns the message and corresponding pose
		* Returns false if it was not possible to read the message or pose
		* True otherwise
		*/
		bool getNextMessage(MessageType &msg_out,  tf::Transform &sensor_pose){
			if(I == view->end()){
				fprintf(stderr,"End of measurement file Reached!!\n");
				return false;
			}
			
			bool gotMessage = false;
			ros::Time ts;
			rosbag::MessageInstance const m = *I;
			//sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
			typename MessageType::ConstPtr msg = m.instantiate<MessageType>();
			
			if(msg != NULL){
				if(m.getTopic() == message_topic){
					msg_out = (*msg);
					ts = msg_out.header.stamp;
					gotMessage = true;
					//fprintf(stderr,"Got Message (ts=%lf)\n",ts.toSec());
				}else{
					//fprintf(stderr,"Topic does not match!\n");
				}
			}else{
				//fprintf(stderr,"msg NULL\n");
			}
			
			readUntilTime(ts+ros::Duration(2.0));
			
			
			if(gotMessage){	
				std::string schaiba;
				schaiba.resize(200);
				if(!transformer.canTransform(fixed_link, target_linkname, ts, &schaiba)){
					fprintf(stderr,"%s\n",schaiba.c_str());
					I++;
					return false;
				}
				
				tf::StampedTransform transform;
				transformer.lookupTransform(fixed_link, target_linkname, ts,transform);
				sensor_pose = transform;
				I++;
				return true;
			}
			
			I++;
			return false;
		}
		
		
		/**
		* Reads the bag file ahead of time in order to make sure that there are poses in cache
		*/
		bool readUntilTime(ros::Time t){
			if(Itf == view->end()){
				return false;
			}
			if(last_read_tf>t){
				//fprintf(stderr,"No need to read!\n");
				return false; ///No need to read new ones
			}
			
			while(last_read_tf <= t && Itf != view->end()){
				rosbag::MessageInstance const m = *Itf;
				//fprintf(stderr,"READING\n");
				//////////////////////////////////////////////////////////////////////////////////
				tf::tfMessage::ConstPtr transform = m.instantiate<tf::tfMessage>();
				if (transform != NULL){
					if(m.getTopic() == "/tf"){
						
						if(transform->transforms.size()<=0) continue;
						
						for (unsigned int i = 0; i < transform->transforms.size(); i++){
								//std::cout << "frame_id : " << transform->transforms[i].header.frame_id;
								//std::cout << " cframe_id : " << transform->transforms[i].child_frame_id << std::endl;
								tf::StampedTransform stf;
								transformStampedMsgToTF(transform->transforms[i], stf);
								transformer.setTransform(stf);
							}
							
							last_read_tf = transform->transforms[0].header.stamp;
					}
				}
				Itf++;
				/////////////////////////////////////////////////////////////////////////////////////////////
			}
			return true;
		}
		
		
		
		rosbag::View *view;
		rosbag::View::iterator I;
	private:
		std::string fixed_link;
		std::string target_linkname;
		std::string message_topic;
		rosbag::Bag bag;	
		rosbag::View::iterator Itf;
		tf::Transformer transformer;
		ros::Time last_read_tf;
};


#endif


