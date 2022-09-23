#include <ros/ros.h>

#include "my_rosplan_interface/takehint_action.h"
#include <unistd.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>
#include <erl2/ErlOracle.h>
#include <string>
#include <vector>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

//Ontology
#include <erl2/onto.h>
#include <erl2/hints.h>

ros::Publisher pub;
ros::Publisher hints_pub;

namespace KCL_rosplan {
	float z0;
	float z1;
	float z2;
	float z3;
	
	bool present = false;

	int ID;
	std::string ID_str;
	std::string key;
	std::string class_1;


	std::vector<std::string> hint_0;
	std::vector<std::string> hint_1;
	std::vector<std::string> hint_2;
	std::vector<std::string> hint_3;
	std::vector<std::string> hint_4;
	std::vector<std::string> hint_5;
	/*
	std::string hint_0[3] = {""};
	std::string hint_1[3] = {""};
	std::string hint_2[3] = {""};
	std::string hint_3[3] = {""};
	std::string hint_4[3] = {""};
	std::string hint_5[3] = {""};
	*/
	void load_ontology (std::string ID, std::string key, std::string class_1);
	
	TakeHintInterface::TakeHintInterface(ros::NodeHandle &nh) {
	// here the initialization
	}
	
	bool TakeHintInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		
		std::cout << "TAKING HINT: from " << msg->parameters[0].value << std::endl;
		
			
		moveit::planning_interface::MoveGroupInterface group("arm");
			group.setEndEffectorLink("cluedo_link");
			group.setPoseReferenceFrame("base_link");
			group.setPlannerId("RRTstar");
			group.setNumPlanningAttempts(10);
			group.setPlanningTime(10.0);
			group.allowReplanning(true);
			group.setGoalJointTolerance(0.0001);
			group.setGoalPositionTolerance(0.0001);
			group.setGoalOrientationTolerance(0.001);
			
		if(msg->parameters[0].value == "wp0"){
			if (z0 == 1.25){
				group.setNamedTarget("1_25_maybe");
				}
			else if (z0 == 0.75){
				group.setNamedTarget("0_75_maybe");
				}
			else{
				printf("Z0 NOT WORKING\n");
				group.setNamedTarget("zero");
				}
			group.move(); 
		}
		
		else if (msg->parameters[0].value == "wp1"){
			if (z1 == 1.25){
				group.setNamedTarget("1_25_maybe");
				}
			else if (z1 == 0.75){
				group.setNamedTarget("0_75_maybe");
				}
			else{
				printf("Z1 NOT WORKING\n");
				group.setNamedTarget("zero");
				}
			group.move(); 
		}
		else if (msg->parameters[0].value == "wp2"){
			if (z2 == 1.25){
				group.setNamedTarget("1_25_maybe");
				}
			else if (z2 == 0.75){
				group.setNamedTarget("0_75_maybe");
				}
			else{
				printf("Z2 NOT WORKING\n");
				group.setNamedTarget("zero");
				}
			group.move(); 
		}
		else if (msg->parameters[0].value == "wp3"){		
			if (z3 == 1.25){
				group.setNamedTarget("1_25_maybe");
				}
			else if (z3 == 0.75){
				group.setNamedTarget("0_75_maybe");
				}
			else{
				printf("Z3 NOT WORKING\n");
				group.setNamedTarget("zero");
				}
			group.move();
		}
		//ac.sendGoal(goal);
		//ac.waitForResult();
	 	else {
	 		std::cout<< "SOMETHING WENT WRONG" << std::endl;
	 	}
	 	
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}
	
	
	void marker_position_callback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
	
		//int id0 = msg->markers[0].id;
		z0 = msg->markers[0].pose.position.z;
		//printf("Marker %d: x: %f, y: %f, z: %f\n", id0,x0,y0,z0);
		
		//int id1 = msg->markers[1].id;
		z1 = msg->markers[1].pose.position.z;
		//printf("Marker %d: x: %f, y: %f, z: %f\n", id1,x1,y1,z1);
		
		//int id2 = msg->markers[2].id;
		z2 = msg->markers[2].pose.position.z;
		//printf("Marker %d: x: %f, y: %f, z: %f\n", id2,x2,y2,z2);
		
		//int id3 = msg->markers[3].id;
		z3 = msg->markers[3].pose.position.z;
		//printf("Marker %d: x: %f, y: %f, z: %f\n", id3,x3,y3,z3);
		
		//std::cout << "id: " << id << std::endl;
		
		//std::cout<< "size(markers): " << msg->markers.size() << std::endl;
	}
	
	void oracle_hint_callback(const erl2::ErlOracle::ConstPtr& msg) {
		
		//Prima di prendere l'indizio nuovo controllo che non sia già di dimensione 3.
		// Se ha dimensione 3 vuol dire che non è consistente (perchè lo ho controllato alla fine del
		//"ciclo" precedente, oppure che sta per diventare di dimensione 4. Comunque non va bene.
		
		if (hint_0.size() == 4){
			std::cout << "Clearing hint_0" << std::endl;
			hint_0.clear();
		}
		if (hint_1.size() == 4){
			std::cout << "Clearing hint_1" << std::endl;
			hint_1.clear();
		}
		if (hint_2.size() == 4){
			std::cout << "Clearing hint_2" << std::endl;
			hint_2.clear();
		}
		if (hint_3.size() == 4){
			std::cout << "Clearing hint_3" << std::endl;
			hint_3.clear();
		}
		if (hint_4.size() == 4){
			std::cout << "Clearing hint_4" << std::endl;
			hint_4.clear();
		}
		if (hint_5.size() == 4){
			std::cout << "Clearing hint_5" << std::endl;
			hint_5.clear();
		}
		
		ID = msg-> ID;
		key = msg-> key;
		class_1 = msg-> value;
		
		std::cout<< "HINT: "<<std::endl;
		std::cout<< "ID: "<< ID <<std::endl;
		std::cout<< "KEY: "<< key <<std::endl;
		std::cout<< "class_1: "<< class_1 <<std::endl;
				
		if (class_1 != "0" && class_1 != "" && class_1 != "-1")
		{
			if(key == "who" || key == "what" || key == "where") {
				
				if (ID == 0) {
					ID_str = "ID0";
					std::string new_hint;
					new_hint.append(key);
					new_hint.append(":");
					new_hint.append(class_1);
					
					for (int i = 0; i < hint_0.size(); i++) {
						if (hint_0[i] == new_hint){
							present = true;
						}
					}
					if (present == false) {
						hint_0.push_back(new_hint);
						std::cout << "ID_str: " << ID_str << std::endl;
						load_ontology(ID_str, key, class_1);
					}
					else{
						std::cout << "Hint already present in hypothesis" << std::endl;
						present = false;
					}
				}
				
				else if (ID == 1) {
					ID_str = "ID1";
					std::string new_hint;
					new_hint.append(key);
					new_hint.append(":");
					new_hint.append(class_1);
					for (int i = 0; i < hint_1.size(); i++) {
						if (hint_1[i] == new_hint){
							present = true;
						}
					}
					if (present == false) { 
						hint_1.push_back(new_hint);
						std::cout << "ID_str: " << ID_str << std::endl;
						load_ontology(ID_str, key, class_1);
					}
					else{
						std::cout << "Hint already present in hypothesis" << std::endl;
						present = false;
					}
				}
				
				else if (ID == 2) {
					ID_str = "ID2";
					std::string new_hint;
					new_hint.append(key);
					new_hint.append(":");
					new_hint.append(class_1);
					for (int i = 0; i < hint_2.size(); i++) {
						if (hint_2[i] == new_hint){
							present = true;
						}
					}
					if (present == false) { 
						hint_2.push_back(new_hint);
						std::cout << "ID_str: " << ID_str << std::endl;
						load_ontology(ID_str, key, class_1);
					}
					else{
						std::cout << "Hint already present in hypothesis" << std::endl;
						present = false;
					}
				}
				
				else if (ID == 3) {
					ID_str = "ID3";
					std::string new_hint;
					new_hint.append(key);
					new_hint.append(":");
					new_hint.append(class_1);
					for (int i = 0; i < hint_3.size(); i++) {
						if (hint_3[i] == new_hint){
							present = true;
						}
					}
					if (present == false) { 
						hint_3.push_back(new_hint);
						std::cout << "ID_str: " << ID_str << std::endl;
						load_ontology(ID_str, key, class_1);
					}
					else{
						std::cout << "Hint already present in hypothesis" << std::endl;
						present = false;
					}
				}
				
				else if (ID == 4) {
					ID_str = "ID4";
					std::string new_hint;
					new_hint.append(key);
					new_hint.append(":");
					new_hint.append(class_1);
					for (int i = 0; i < hint_4.size(); i++) {
						if (hint_4[i] == new_hint){
							present = true;
						}
					}
					if (present == false) { 
						hint_4.push_back(new_hint);
						std::cout << "ID_str: " << ID_str << std::endl;
						load_ontology(ID_str, key, class_1);
					}
					else{
						std::cout << "Hint already present in hypothesis" << std::endl;
						present = false;
					}
				}
				
				else if (ID == 5) {
					ID_str = "ID5";
					std::string new_hint;
					new_hint.append(key);
					new_hint.append(":");
					new_hint.append(class_1);
					for (int i = 0; i < hint_5.size(); i++) {
						if (hint_5[i] == (new_hint)){
							present = true;
						}
					}
					if (present == false) { 
						hint_5.push_back(new_hint);
						std::cout << "ID_str: " << ID_str << std::endl;
						load_ontology(ID_str, key, class_1);
					}
					else{
						std::cout << "Hint already present in hypothesis" << std::endl;
						present = false;
					}
				}
							
				else {
					std::cout<< "ID error" << std::endl;
				}
				
				erl2::hints hint_msg;
		
				hint_msg.hint_0 = hint_0;
				hint_msg.hint_1 = hint_1;
				hint_msg.hint_2 = hint_2;
				hint_msg.hint_3 = hint_3;
				hint_msg.hint_4 = hint_4;
				hint_msg.hint_5 = hint_5;
				
				hints_pub.publish(hint_msg);
				/*
				std::cout << "hint_0 :" << hint_0[0] << " " << hint_0[1] << " " << hint_0[2] << std::endl;
				std::cout << "hint_1 :" << hint_1[0] << " " << hint_1[1] << " " << hint_1[2] << std::endl;
				std::cout << "hint_2 :" << hint_2[0] << " " << hint_2[1] << " " << hint_2[2] << std::endl;
				std::cout << "hint_3 :" << hint_3[0] << " " << hint_3[1] << " " << hint_3[2] << std::endl;
				std::cout << "hint_4 :" << hint_4[0] << " " << hint_4[1] << " " << hint_4[2] << std::endl;
				std::cout << "hint_5 :" << hint_5[0] << " " << hint_5[1] << " " << hint_5[2] << std::endl;
				*/
			}
			else {
				std::cout<< "Key error" << std::endl;
			}
			
		}
		else{
			std::cout<< "Value error" << std::endl;
		}
	}
	
	void load_ontology (std::string ID_str, std::string key, std::string class_1){
		erl2::onto onto_msg;
		onto_msg.class_1 = class_1;
		onto_msg.ID = ID_str;
		std::cout<< "LOAD ONTOLOGY ID: "<< ID_str << " class_1: " << class_1 << std::endl;
		//std::cout <<"ONTOLOGY ID: " << ID << std::endl;
		
		if (key == "who"){
			onto_msg.to_do = 1;
			pub.publish(onto_msg);
		}
		else if (key == "what"){
			onto_msg.to_do = 2;
			pub.publish(onto_msg);
		}
		else if (key == "where"){
			onto_msg.to_do = 3;
			pub.publish(onto_msg);
		}
		else {
			std::cout<< "Key error" << std::endl;
		}
	}
	
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "takehintaction", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	ros::NodeHandle nh1;
	ros::Subscriber sub = nh1.subscribe("/visualization_marker", 10, KCL_rosplan::marker_position_callback);
	ros::Subscriber hint_sub = nh1.subscribe("/oracle_hint", 10, KCL_rosplan::oracle_hint_callback);
	
	hints_pub = nh1.advertise<erl2::hints>("/hint_list", 0);
	
	//Ontology
	pub = nh1.advertise<erl2::onto>("/ontology", 0);
	
	KCL_rosplan::TakeHintInterface my_aci(nh);
	my_aci.runActionInterface();
	
	//Moveit
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	return 0;
}

