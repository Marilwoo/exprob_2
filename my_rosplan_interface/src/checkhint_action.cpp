#include <ros/ros.h>

#include "my_rosplan_interface/checkhint_action.h"
#include <unistd.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>
#include <erl2/ErlOracle.h>
#include <string>

#include <erl2/hints.h>
#include <erl2/Oracle.h>

ros::Publisher pub;

namespace KCL_rosplan {

	std::vector<std::string> hint_0;
	std::vector<std::string> hint_1;
	std::vector<std::string> hint_2;
	std::vector<std::string> hint_3;
	std::vector<std::string> hint_4;
	std::vector<std::string> hint_5;
	
	std::size_t pos = 0;
	std::size_t pos2 = 0;
	std::size_t pos3 = 0;
	std::string key = "";
	std::string key2 = "";
	std::string key3 = "";
	std::string hint = "";
	std::string hint2 = "";
	std::string hint3 = "";

	bool winning = false;
		
	void check_consistent();
	void check_winning(std::string ID);

	CheckHintInterface::CheckHintInterface(ros::NodeHandle &nh) {
	// here the initialization
	}
	
	bool CheckHintInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		KCL_rosplan::check_consistent();
		if (winning == 0 || winning == false){
			return false;
		}
		if (winning == 1 || winning == true){
			return true;
		}
	}
		
	void hint_callback(const erl2::hints::ConstPtr& msg){
		hint_0 = msg->hint_0; 
		hint_1 = msg->hint_1;
		hint_2 = msg->hint_2;
		hint_3 = msg->hint_3;
		hint_4 = msg->hint_4;
		hint_5 = msg->hint_5;
		
	}

	void check_consistent(){
	
		pos = 0;
		pos2 = 0;
		pos3 = 0;
		key = "";
		key2 = "";
		key3 = "";
		hint = "";
		hint2 = "";
		hint3 = "";

		//Controllo su ID0
		if(hint_0.size() == 3){
			//prendere il primo indizio di hint_0
			pos = hint_0[0].find(":");
			key = hint_0[0].substr(0, pos);
			hint = hint_0[0].substr(pos+1);
			//secondo indizio
			pos2 = hint_0[1].find(":");
			key2 = hint_0[1].substr(0, pos2);
			hint2 = hint_0[1].substr(pos2+1);
			//terzo indizio
			pos3 = hint_0[2].find(":");
			key3 = hint_0[2].substr(0, pos3);
			hint3 = hint_0[2].substr(pos3+1);
			if (key != key2 && key2 != key3 && key != key3){
				std::cout << "hypothesis consistent: " << hint << ", "<< hint2 << ", "<< hint3 << std::endl;
				std::cout << "checking if it is the winning one" << std::endl;
				KCL_rosplan::check_winning("0");
			}
		}
		
		//Controllo su ID1
		if(hint_1.size() == 3){
			//prendere il primo indizio di hint_0
			pos = hint_1[0].find(":");
			key = hint_1[0].substr(0, pos);
			hint = hint_1[0].substr(pos+1);
			//secondo indizio
			pos2 = hint_1[1].find(":");
			key2 = hint_1[1].substr(0, pos2);
			hint2 = hint_1[1].substr(pos2+1);
			//terzo indizio
			pos3 = hint_1[2].find(":");
			key3 = hint_1[2].substr(0, pos3);
			hint3 = hint_1[2].substr(pos3+1);
			if (key != key2 && key2 != key3 && key != key3){
				std::cout << "hypothesis consistent: " << hint << ", "<< hint2 << ", "<< hint3 << std::endl;
				std::cout << "checking if it is the winning one" << std::endl;
				KCL_rosplan::check_winning("1");
			}
		}
		
		//Controllo su ID2
		if(hint_2.size() == 3){
			//prendere il primo indizio di hint_0
			pos = hint_2[0].find(":");
			key = hint_2[0].substr(0, pos);
			hint = hint_2[0].substr(pos+1);
			//secondo indizio
			pos2 = hint_2[1].find(":");
			key2 = hint_2[1].substr(0, pos2);
			hint2 = hint_2[1].substr(pos2+1);
			//terzo indizio
			pos3 = hint_2[2].find(":");
			key3 = hint_2[2].substr(0, pos3);
			hint3 = hint_2[2].substr(pos3+1);
			if (key != key2 && key2 != key3 && key != key3){
				std::cout << "hypothesis consistent: " << hint << ", "<< hint2 << ", "<< hint3 << std::endl;
				std::cout << "checking if it is the winning one" << std::endl;
				KCL_rosplan::check_winning("2");
			}
		}
		
		//Controllo su ID3
		if(hint_3.size() == 3){
			//prendere il primo indizio di hint_0
			pos = hint_3[0].find(":");
			key = hint_3[0].substr(0, pos);
			hint = hint_3[0].substr(pos+1);
			//secondo indizio
			pos2 = hint_3[1].find(":");
			key2 = hint_3[1].substr(0, pos2);
			hint2 = hint_3[1].substr(pos2+1);
			//terzo indizio
			pos3 = hint_3[2].find(":");
			key3 = hint_3[2].substr(0, pos3);
			hint3 = hint_3[2].substr(pos3+1);
			if (key != key2 && key2 != key3 && key != key3){
				std::cout << "hypothesis consistent: " << hint << ", "<< hint2 << ", "<< hint3 << std::endl;
				std::cout << "checking if it is the winning one" << std::endl;
				KCL_rosplan::check_winning("3");
			}
		}
		
		//Controllo su ID4
		if(hint_4.size() == 3){
			//prendere il primo indizio di hint_0
			pos = hint_4[0].find(":");
			key = hint_4[0].substr(0, pos);
			hint = hint_4[0].substr(pos+1);
			//secondo indizio
			pos2 = hint_4[1].find(":");
			key2 = hint_4[1].substr(0, pos2);
			hint2 = hint_4[1].substr(pos2+1);
			//terzo indizio
			pos3 = hint_4[2].find(":");
			key3 = hint_4[2].substr(0, pos3);
			hint3 = hint_4[2].substr(pos3+1);
			if (key != key2 && key2 != key3 && key != key3){
				std::cout << "hypothesis consistent: " << hint << ", "<< hint2 << ", "<< hint3 << std::endl;
				std::cout << "checking if it is the winning one" << std::endl;
				KCL_rosplan::check_winning("4");
			}
		}
		
		//Controllo su ID5
		if(hint_5.size() == 3){
			//prendere il primo indizio di hint_0
			pos = hint_5[0].find(":");
			key = hint_5[0].substr(0, pos);
			hint = hint_5[0].substr(pos+1);
			//secondo indizio
			pos2 = hint_5[1].find(":");
			key2 = hint_5[1].substr(0, pos2);
			hint2 = hint_5[1].substr(pos2+1);
			//terzo indizio
			pos3 = hint_5[2].find(":");
			key3 = hint_5[2].substr(0, pos3);
			hint3 = hint_5[2].substr(pos3+1);
			if (key != key2 && key2 != key3 && key != key3){
				std::cout << "hypothesis consistent: " << hint << ", "<< hint2 << ", "<< hint3 << std::endl;
				std::cout << "checking if it is the winning one" << std::endl;
				KCL_rosplan::check_winning("5");
			}
		}
	}
	
	void check_winning(std::string ID){
		ros::NodeHandle nh3;
		ros::ServiceClient win = nh3.serviceClient<erl2::Oracle>("/oracle_solution");
		erl2::Oracle srv;
		win.call(srv);
		std::string win_ID = std::to_string(srv.response.ID);
		
		std::string who;
		std::string what;
		std::string where;
		
		if (key == "who"){
			who = hint;
		}
		else if (key2 == "who"){
			who = hint2;
		}
		else if (key3 == "who"){
			who = hint3;
		}
		if (key == "what"){
			what = hint;
		}
		else if (key2 == "what"){
			what = hint2;
		}
		else if (key3 == "what"){
			what = hint3;
		}
		if (key == "where"){
			where = hint;
		}
		else if (key2 == "where"){
			where = hint2;
		}
		else if (key3 == "where"){
			where = hint3;
		}
		
		std::cout << "My hypothesis: It was " << who << " with a " << what << " in the " << where << std::endl;		
		
		if (win_ID == ID){
			std::cout << "Correct hypothesis" << std::endl;
			std::cout << "I won!!!" << std::endl;
			winning = true;
		}
		else{
			std::cout << "Not winning hypothesis, searching for another one" << std::endl;
			winning = false;
		}
	
	}
}
	
int main(int argc, char **argv) {

	ros::init(argc, argv, "checkhintaction", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	ros::NodeHandle nh1;
	
	ros::Subscriber hints = nh1.subscribe("/hint_list", 0, KCL_rosplan::hint_callback);
	
	//Ontology
	pub = nh1.advertise<erl2::onto>("/ontology", 0);
	
	KCL_rosplan::CheckHintInterface my_aci(nh);
	my_aci.runActionInterface();
	
	return 0;
}


