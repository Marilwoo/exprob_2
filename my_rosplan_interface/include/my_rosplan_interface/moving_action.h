#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"
#include "visualization_msgs/MarkerArray.h"
namespace KCL_rosplan {

class MovingActionInterface: public RPActionInterface
	{
		private:
		
		public:
		
			/* constructor */
			MovingActionInterface(ros::NodeHandle &nh);
			/* listen to and process action_dispatch topic */
			bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}

