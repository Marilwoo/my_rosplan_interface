#include <ros/ros.h>

#include "my_rosplan_interface/checkhint_action.h"
#include <unistd.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>
#include <erl2/ErlOracle.h>
#include <string>

//Ontology
#include <erl2/hints.h>
#include <erl2/onto.h>
//#include <erl2/check_msg.h>
#include <erl2/Check_srv.h>
#include <erl2/Oracle.h>

ros::Publisher pub;
//ros::ServiceClient check_client;
//ros::ServiceClient win;

namespace KCL_rosplan {

	std::vector<std::string> hint_0;
	std::vector<std::string> hint_1;
	std::vector<std::string> hint_2;
	std::vector<std::string> hint_3;
	std::vector<std::string> hint_4;
	std::vector<std::string> hint_5;

	bool check = false;
	bool winning = false;
		
	//void check_dimension();
	void check_consistent();
	void check_winning(std::string ID);

	CheckHintInterface::CheckHintInterface(ros::NodeHandle &nh) {
	// here the initialization
	}
	
	bool CheckHintInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
	
		//KCL_rosplan::check_dimension();
		KCL_rosplan::check_consistent();

		
		//std::cout<<"CHECK_HINT ACTION CHECK: "<< check << std::endl;
		
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
	/*
	void check_dimension(){
		if (hint_0.size() == 3){
			std::cout<< "ID0 has complete hypothesis, checking if consistent"<< std::endl;
			check_consistent("ID0");
		}
		else if (hint_1.size() == 3){
			std::cout<< "ID1 has complete hypothesis, checking if consistent"<< std::endl;
			check_consistent("ID1");
		}
		else if (hint_2.size() == 3){
			std::cout<< "ID2 has complete hypothesis, checking if consistent"<< std::endl;
			check_consistent("ID2");
		}
		else if (hint_3.size() == 3){
			std::cout<< "ID3 has complete hypothesis, checking if consistent"<< std::endl;
			check_consistent("ID3");
		}
		else if (hint_4.size() == 3){
			std::cout<< "ID4 has complete hypothesis, checking if consistent"<< std::endl;
			check_consistent("ID4");
		}
		else if (hint_5.size() == 3){
			std::cout<< "ID5 has complete hypothesis, checking if consistent"<< std::endl;
			check_consistent("ID5");
		}
		else {
		std::cout << "CHECK HINT: No complete hypotheses" << std::endl;
		check = false;
		}
	}		
	*/
	
	void check_consistent(){
		std::size_t pos = 0;
		std::size_t pos2 = 0;
		std::size_t pos3 = 0;
		std::string key = "";
		std::string key2 = "";
		std::string key3 = "";
		std::string hint = "";
		std::string hint2 = "";
		std::string hint3 = "";
		//std::cout << "In check consistent. Checking: "<< ID << std::endl;
		/*ros::NodeHandle nh2;
		ros::ServiceClient check_client = nh2.serviceClient<erl2::Check_srv>("/check");
		erl2::Check_srv srv;
		srv.request.ID_srv = ID;
		check_client.call(srv);
		
		check = srv.response.check;
		*/
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
			
			if (key != key2 && key2 != key3 && key1 != key3){
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
			
			if (key != key2 && key2 != key3 && key1 != key3){
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
			
			if (key != key2 && key2 != key3 && key1 != key3){
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
			
			if (key != key2 && key2 != key3 && key1 != key3){
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
			
			if (key != key2 && key2 != key3 && key1 != key3){
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
			
			if (key != key2 && key2 != key3 && key1 != key3){
				std::cout << "hypothesis consistent: " << hint << ", "<< hint2 << ", "<< hint3 << std::endl;
				std::cout << "checking if it is the winning one" << std::endl;
				KCL_rosplan::check_winning("5");
			}
		}
		
		/*
		if (check == false) {
			std::cout << ID <<" is not consistent, searching for another hypothesis." << std::endl;
		}
		else{
			std::cout << ID <<" is consistent, checking if it is the winning hypothesis" << std::endl;
			KCL_rosplan::check_winning(0);
		}
		
		
		erl2::onto onto_msg;
		onto_msg.to_do = 4;
		onto_msg.ID = ID;
		onto_msg.class_1 = "";
		pub.publish(onto_msg);
		std::cout<< "check in consistent: " << check << std::endl;
		if (check == true){
			std::cout<< "Hypothesis consistent"<< std::endl;
		}
		else{
			std::cout <<"Hypothesis NOT consistent"<< std::endl;
		}
		*/
	}
	
	void check_winning(std::string ID){
		ros::NodeHandle nh3;
		ros::ServiceClient win = nh3.serviceClient<erl2::Oracle>("/oracle_solution");
		erl2::Oracle srv;
		win.call(srv);
		std::string win_ID = std::to_string(srv.response.ID);
		
		if (win_ID == ID){
			std::cout << "winning hypothesis: " << ID << std::endl;
			winning = true;
		}
		else{
			std::cout << "Not winning hypothesis, searching for another one" << std::endl;
			winning = false;
		}
	
	}
	/*
	void hint_check_callback(const erl2::check_msg::ConstPtr& msg){
		check = msg -> check;
		std::cout << "check_hint action check: " << check<< std::endl;
	}
	*/
}
	
int main(int argc, char **argv) {

	ros::init(argc, argv, "checkhintaction", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	ros::NodeHandle nh1;
	
	ros::Subscriber hints = nh1.subscribe("/hint_list", 0, KCL_rosplan::hint_callback);
	
	//Ontology
	pub = nh1.advertise<erl2::onto>("/ontology", 0);
	//TODO da togliere
	//ros::Subscriber check_sub = nh1.subscribe("/check", 10, KCL_rosplan::hint_check_callback);
	
	KCL_rosplan::CheckHintInterface my_aci(nh);
	my_aci.runActionInterface();
	
	return 0;
}


