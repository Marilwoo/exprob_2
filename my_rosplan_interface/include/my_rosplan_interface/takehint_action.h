#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"
#include "visualization_msgs/MarkerArray.h"
#include "erl2/ErlOracle.h"
#include "erl2/onto.h"
#include "erl2/check_msg.h"
#include <string>
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
			void oracle_hint_callback(const erl2::ErlOracle::ConstPtr& msg);
	};
}

