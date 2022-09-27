/**
* \file checkhint_action.cpp
* \brief Node for managing the hints: checking if there are complete and consistent hypoteheses anche checking the winning one.
* \author Maria Luisa Aiachini
*
* \details
*
* Clients: <BR>
*	°/oracle_solution
*
* Subscribes to: <BR>
*	°/hint_list
*
* Description:
*
* This node is used to check the hints. It takes the hypotheses list, through /hint_list topic, it checks if there is
* any complete and consistent hypotheses, if so, it checks if it is the winning one, after printing it in natural language. 
*
*/

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

	//Initiating variables
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

/**
* \param msg
*
* \return true if the winning hypothesis is found; false otherwise
*
* This is the callback of the action. It calls the function check_consistent() that checks if there are any 
* complete and consistent hypotheses. It then returns true if the winning hypothesis is the winning one, and 
* returns false otherwise.
*
*/	
	bool CheckHintInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		//Calling the function
		KCL_rosplan::check_consistent();
		//Returns the success or not of the action depending if the winning hyothesis is found
		if (winning == 0 || winning == false){
			return false;
		}
		if (winning == 1 || winning == true){
			return true;
		}
	}
	
/**
* \param msg: containing the hypotheses list
*
* Callback function for the /hint_list subscriber
*
*/		
	void hint_callback(const erl2::hints::ConstPtr& msg){
		hint_0 = msg->hint_0; 
		hint_1 = msg->hint_1;
		hint_2 = msg->hint_2;
		hint_3 = msg->hint_3;
		hint_4 = msg->hint_4;
		hint_5 = msg->hint_5;
		
	}

/**
* Function for checking the completeness and the consistency of all the hypotheses.
* It first check if one hypothesis is complete (dimension 3) it then retreives the values and the corresponding keys.
* If the hypothesis is found complete and consistent it calls the check_winning() function.
*
*/
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

		//ID0 check
		if(hint_0.size() == 3){
			//Taking the first hint of the hypothesis
			pos = hint_0[0].find(":");
			key = hint_0[0].substr(0, pos);
			hint = hint_0[0].substr(pos+1);
			//second hint
			pos2 = hint_0[1].find(":");
			key2 = hint_0[1].substr(0, pos2);
			hint2 = hint_0[1].substr(pos2+1);
			//third hint
			pos3 = hint_0[2].find(":");
			key3 = hint_0[2].substr(0, pos3);
			hint3 = hint_0[2].substr(pos3+1);
			//Checking if it is consistent
			if (key != key2 && key2 != key3 && key != key3){
				std::cout << '\n'<< '\n';
				std::cout << "hypothesis consistent: " << hint << ", "<< hint2 << ", "<< hint3 << std::endl;
				std::cout << "checking if it is the winning one" << std::endl;
				std::cout << '\n'<< '\n';
				//Checking if it is the winning one
				KCL_rosplan::check_winning("0");
			}
		}
		
		//ID1 check
		if(hint_1.size() == 3){
			pos = hint_1[0].find(":");
			key = hint_1[0].substr(0, pos);
			hint = hint_1[0].substr(pos+1);
			
			pos2 = hint_1[1].find(":");
			key2 = hint_1[1].substr(0, pos2);
			hint2 = hint_1[1].substr(pos2+1);
			
			pos3 = hint_1[2].find(":");
			key3 = hint_1[2].substr(0, pos3);
			hint3 = hint_1[2].substr(pos3+1);
			
			if (key != key2 && key2 != key3 && key != key3){
				std::cout << '\n'<< '\n';
				std::cout << "hypothesis consistent: " << hint << ", "<< hint2 << ", "<< hint3 << std::endl;
				std::cout << "checking if it is the winning one" << std::endl;
				std::cout << '\n'<< '\n';
				KCL_rosplan::check_winning("1");
			}
		}
		
		//ID2 check
		if(hint_2.size() == 3){
			pos = hint_2[0].find(":");
			key = hint_2[0].substr(0, pos);
			hint = hint_2[0].substr(pos+1);
			
			pos2 = hint_2[1].find(":");
			key2 = hint_2[1].substr(0, pos2);
			hint2 = hint_2[1].substr(pos2+1);
			
			pos3 = hint_2[2].find(":");
			key3 = hint_2[2].substr(0, pos3);
			hint3 = hint_2[2].substr(pos3+1);
			
			if (key != key2 && key2 != key3 && key != key3){
				std::cout << '\n'<< '\n';
				std::cout << "hypothesis consistent: " << hint << ", "<< hint2 << ", "<< hint3 << std::endl;
				std::cout << "checking if it is the winning one" << std::endl;
				std::cout << '\n'<< '\n';
				KCL_rosplan::check_winning("2");
			}
		}
		
		//ID3 check
		if(hint_3.size() == 3){
			pos = hint_3[0].find(":");
			key = hint_3[0].substr(0, pos);
			hint = hint_3[0].substr(pos+1);

			pos2 = hint_3[1].find(":");
			key2 = hint_3[1].substr(0, pos2);
			hint2 = hint_3[1].substr(pos2+1);

			pos3 = hint_3[2].find(":");
			key3 = hint_3[2].substr(0, pos3);
			hint3 = hint_3[2].substr(pos3+1);
			
			if (key != key2 && key2 != key3 && key != key3){
				std::cout << '\n'<< '\n';
				std::cout << "hypothesis consistent: " << hint << ", "<< hint2 << ", "<< hint3 << std::endl;
				std::cout << "checking if it is the winning one" << std::endl;
				std::cout << '\n'<< '\n';
				KCL_rosplan::check_winning("3");
			}
		}
		
		//ID4 check
		if(hint_4.size() == 3){
			pos = hint_4[0].find(":");
			key = hint_4[0].substr(0, pos);
			hint = hint_4[0].substr(pos+1);

			pos2 = hint_4[1].find(":");
			key2 = hint_4[1].substr(0, pos2);
			hint2 = hint_4[1].substr(pos2+1);

			pos3 = hint_4[2].find(":");
			key3 = hint_4[2].substr(0, pos3);
			hint3 = hint_4[2].substr(pos3+1);
			
			if (key != key2 && key2 != key3 && key != key3){
				std::cout << '\n'<< '\n';
				std::cout << "hypothesis consistent: " << hint << ", "<< hint2 << ", "<< hint3 << std::endl;
				std::cout << "checking if it is the winning one" << std::endl;
				std::cout << '\n'<< '\n';
				KCL_rosplan::check_winning("4");
			}
		}
		
		//ID5 check
		if(hint_5.size() == 3){
			pos = hint_5[0].find(":");
			key = hint_5[0].substr(0, pos);
			hint = hint_5[0].substr(pos+1);

			pos2 = hint_5[1].find(":");
			key2 = hint_5[1].substr(0, pos2);
			hint2 = hint_5[1].substr(pos2+1);

			pos3 = hint_5[2].find(":");
			key3 = hint_5[2].substr(0, pos3);
			hint3 = hint_5[2].substr(pos3+1);
			
			if (key != key2 && key2 != key3 && key != key3){
				std::cout << '\n'<< '\n';
				std::cout << "hypothesis consistent: " << hint << ", "<< hint2 << ", "<< hint3 << std::endl;
				std::cout << "checking if it is the winning one" << std::endl;
				std::cout << '\n'<< '\n';
				KCL_rosplan::check_winning("5");
			}
		}
	}

/**
* \param ID: containing the ID of the complete and consistent hypothesis
*
* In this function there is the initialization of the client of type Oracle, with the service /oracle_solution.
* On this server is written the winning ID. In this funcion the server is called. Once received the response with the
* winning ID it is compared with the one of the hypotehsis that is being checked.
* In this function also the structure of the hypothesis is retreived (values and corresponding keys) in order to print
* the hypothesis in natural language.
*/	
	void check_winning(std::string ID){
		ros::NodeHandle nh3;
		
		//Initializing the client
		ros::ServiceClient win = nh3.serviceClient<erl2::Oracle>("/oracle_solution");
		erl2::Oracle srv;
		
		//Calling the service
		win.call(srv);
		std::string win_ID = std::to_string(srv.response.ID);
		
		std::string who;
		std::string what;
		std::string where;
		
		//Retreiving corresponding keys of the hints
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
		
		//Printing the hypothesis in natural language
		std::cout << "My hypothesis: It was " << who << " with a " << what << " in the " << where << std::endl;		
		
		//Winning hypothesis case
		if (win_ID == ID){
			std::cout << "Correct hypothesis" << std::endl;
			std::cout << "I won!!!" << std::endl;
			winning = true;
		}
		//Not winning hypothesis case
		else{
			std::cout << "Not winning hypothesis, searching for another one" << std::endl;
			winning = false;
		}
	
	}
}

/**
* \param argc
* \param argv
*
* \return always 0
*
* Main function for the checkhintaction node.
* Here the initialization of the node and of the action for rosplan.
* Also it is initialized the /hint_list subscriber
*
*
*/	
int main(int argc, char **argv) {

	ros::init(argc, argv, "checkhintaction", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	ros::NodeHandle nh1;
	
	ros::Subscriber hints = nh1.subscribe("/hint_list", 0, KCL_rosplan::hint_callback);
	
	KCL_rosplan::CheckHintInterface my_aci(nh);
	my_aci.runActionInterface();
	
	return 0;
}


