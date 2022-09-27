#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"
#include "visualization_msgs/MarkerArray.h"
#include "erl2/ErlOracle.h"
#include "erl2/check_msg.h"
#include "erl2/onto.h"
#include "erl2/hints.h"
#include <string>
namespace KCL_rosplan {

class CheckHintInterface: public RPActionInterface
	{
		private:
		
		public:
		
			/* constructor */
			CheckHintInterface(ros::NodeHandle &nh);
			/* listen to and process action_dispatch topic */
			bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
			//void check_dimension();
			void check_consistent();
			//void hint_check_callback(const erl2::check_msg::ConstPtr& msg);
			void hint_callback(const erl2::hints::ConstPtr& msg);
			void check_winning(std::string ID);
	};
}

