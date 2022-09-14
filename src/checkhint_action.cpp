#include <ros/ros.h>

#include "my_rosplan_interface/checkhint_action.h"
#include <unistd.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>
#include <erl2/ErlOracle.h>

//Ontology
#include <erl2/hints.h>
#include <erl2/onto.h>
#include <erl2/check_msg.h>

ros::Publisher pub;

namespace KCL_rosplan {

	std::vector<std::string> hint_0(3);
	std::vector<std::string> hint_1(3);
	std::vector<std::string> hint_2(3);
	std::vector<std::string> hint_3(3);
	std::vector<std::string> hint_4(3);
	std::vector<std::string> hint_5(3);

	bool check = false;
		
	void check_dimension();
	void check_consistent(int ID);

	CheckHintInterface::CheckHintInterface(ros::NodeHandle &nh) {
	// here the initialization
	}
	
	bool CheckHintInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		
		//std::cout<< "check: "<< check << std::endl;
		if (check == 0 || check == false){
			return false;
		}
		if (check == 1 || check == true){
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
		
		check_dimension();
	}
	
	void check_dimension(){
		if (hint_0[0] != "" && hint_0[1] !="" && hint_0[2] != ""){
			std::cout<< "ID0 has complete hypothesis, checking if consistent"<< std::endl;
			check_consistent(0);
		}
		else if (hint_1[0] != "" && hint_1[1] !="" && hint_1[2] != ""){
			std::cout<< "ID1 has complete hypothesis, checking if consistent"<< std::endl;
			check_consistent(1);
		}
		else if (hint_2[0] != "" && hint_2[1] !="" && hint_2[2] != ""){
			std::cout<< "ID2 has complete hypothesis, checking if consistent"<< std::endl;
			check_consistent(2);
		}
		else if (hint_3[0] != "" && hint_3[1] !="" && hint_3[2] != ""){
			std::cout<< "ID3 has complete hypothesis, checking if consistent"<< std::endl;
			check_consistent(3);
		}
		else if (hint_4[0] != "" && hint_4[1] !="" && hint_4[2] != ""){
			std::cout<< "ID4 has complete hypothesis, checking if consistent"<< std::endl;
			check_consistent(4);
		}
		else if (hint_5[0] != "" && hint_5[1] !="" && hint_5[2] != ""){
			std::cout<< "ID5 has complete hypothesis, checking if consistent"<< std::endl;
			check_consistent(5);
		}
		else {
		std::cout << "No complete hypotheses" << std::endl;
		}
	}		
	
	void check_consistent(int ID){
		erl2::onto onto_msg;
		onto_msg.to_do = 4;
		onto_msg.ID = ID;
		onto_msg.value = "";
		pub.publish(onto_msg);
		std::cout<< "check in consistent: " << check << std::endl;
		if (check == true){
			std::cout<< "Hypothesis consistent"<< std::endl;
		}
		else{
			std::cout <<"Hypothesis NOT consistent"<< std::endl;
		}
	
	}
	
	void hint_check_callback(const erl2::check_msg::ConstPtr& msg){
		check = msg -> check;
		std::cout << "check: " << check<< std::endl;
	}
}
	
int main(int argc, char **argv) {

	ros::init(argc, argv, "checkhintaction", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	ros::NodeHandle nh1;
	
	ros::Subscriber hints_sub = nh1.subscribe("/hint_list", 0, KCL_rosplan::hint_callback);
	
	//Ontology
	pub = nh1.advertise<erl2::onto>("/ontology", 0);
	ros::Subscriber check_sub = nh1.subscribe("/check", 10, KCL_rosplan::hint_check_callback);
	
	KCL_rosplan::CheckHintInterface my_aci(nh);
	my_aci.runActionInterface();
	
	return 0;
}


