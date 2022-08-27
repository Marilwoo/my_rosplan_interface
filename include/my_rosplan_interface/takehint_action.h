#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"
#include "visualization_msgs/MarkerArray.h"
namespace KCL_rosplan {

class TakeHintInterface: public RPActionInterface
	{
		private:
		
		public:
		
			/* constructor */
			TakeHintInterface(ros::NodeHandle &nh);
			/* listen to and process action_dispatch topic */
			bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
			void marker_position_callback(const visualization_msgs::MarkerArray::ConstPtr& msg);
	};
}

