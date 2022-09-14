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

	
	int ID;
	std::string key;
	std::string value;


	std::vector<std::string> hint_0(3);
	std::vector<std::string> hint_1(3);
	std::vector<std::string> hint_2(3);
	std::vector<std::string> hint_3(3);
	std::vector<std::string> hint_4(3);
	std::vector<std::string> hint_5(3);
	/*
	std::string hint_0[3] = {""};
	std::string hint_1[3] = {""};
	std::string hint_2[3] = {""};
	std::string hint_3[3] = {""};
	std::string hint_4[3] = {""};
	std::string hint_5[3] = {""};
	*/
	void load_ontology (int ID, std::string key, std::string value);
	
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
		ID = msg-> ID;
		key = msg-> key;
		value = msg-> value;
				
		if (value != "0" || value != ""|| value != "-1"){
		
			if (ID == 0){
				if (key == "who"){
				hint_0[0] = value;
				load_ontology(ID, key, value);
				}
				else if (key == "what"){
				hint_0[1] = value;
				load_ontology(ID, key, value);
				}
				else if (key == "where"){
				hint_0[2] = value;
				load_ontology(ID, key, value);
				}
				else {
					std::cout<< "Key error" << std::endl;
				}
			}
			else if (ID == 1){
			if (key == "who"){
				hint_1[0] = value;
				load_ontology(ID, key, value);
				}
				else if (key == "what"){
				hint_1[1] = value;
				load_ontology(ID, key, value);
				}
				else if (key == "where"){
				hint_1[2] = value;
				load_ontology(ID, key, value);
				}
				else {
					std::cout<< "Key error" << std::endl;
				}
			}
			else if (ID == 2){
			if (key == "who"){
				hint_2[0] = value;
				load_ontology(ID, key, value);
				}
				else if (key == "what"){
				hint_2[1] = value;
				load_ontology(ID, key, value);
				}
				else if (key == "where"){
				hint_2[2] = value;
				load_ontology(ID, key, value);
				}
				else {
					std::cout<< "Key error" << std::endl;
				}
			}
			else if (ID == 3){
			if (key == "who"){
				hint_3[0] = value;
				load_ontology(ID, key, value);
				}
				else if (key == "what"){
				hint_3[1] = value;
				load_ontology(ID, key, value);
				}
				else if (key == "where"){
				hint_3[2] = value;
				load_ontology(ID, key, value);
				}
				else {
					std::cout<< "Key error" << std::endl;
				}
			}
			else if (ID == 4){
			if (key == "who"){
				hint_4[0] = value;
				load_ontology(ID, key, value);
				}
				else if (key == "what"){
				hint_4[1] = value;
				load_ontology(ID, key, value);
				}
				else if (key == "where"){
				hint_4[2] = value;
				load_ontology(ID, key, value);
				}
				else {
					std::cout<< "Key error" << std::endl;
				}
			}
			else if (ID == 5){
			if (key == "who"){
				hint_5[0] = value;
				load_ontology(ID, key, value);
				}
				else if (key == "what"){
				hint_5[1] = value;
				load_ontology(ID, key, value);
				}
				else if (key == "where"){
				hint_5[2] = value;
				load_ontology(ID, key, value);
				}
				else {
					std::cout<< "Key error" << std::endl;
				}
			}
			else{
				std::cout<< "ID error" << std::endl;
			}
			
			
			std::cout<< "HINT: "<<std::endl;
			std::cout<< "ID: "<< ID <<std::endl;
			std::cout<< "KEY: "<< key <<std::endl;
			std::cout<< "VALUE: "<< value <<std::endl;
			
			
			erl2::hints hint_msg;
			
			hint_msg.hint_0 = hint_0;
			hint_msg.hint_1 = hint_1;
			hint_msg.hint_2 = hint_2;
			hint_msg.hint_3 = hint_3;
			hint_msg.hint_4 = hint_4;
			hint_msg.hint_5 = hint_5;
			
			hints_pub.publish(hint_msg);
			
			
			std::cout << "hint_0 :" << hint_0[0] << " " << hint_0[1] << " " << hint_0[2] << std::endl;
			std::cout << "hint_1 :" << hint_1[0] << " " << hint_1[1] << " " << hint_1[2] << std::endl;
			std::cout << "hint_2 :" << hint_2[0] << " " << hint_2[1] << " " << hint_2[2] << std::endl;
			std::cout << "hint_3 :" << hint_3[0] << " " << hint_3[1] << " " << hint_3[2] << std::endl;
			std::cout << "hint_4 :" << hint_4[0] << " " << hint_4[1] << " " << hint_4[2] << std::endl;
			std::cout << "hint_5 :" << hint_5[0] << " " << hint_5[1] << " " << hint_5[2] << std::endl;
			
		}
		else{
		std::cout<< "Value error" << std::endl;
		}
	}		

	
	
	void load_ontology (int ID, std::string key, std::string value){
		erl2::onto onto_msg;
		onto_msg.value = value;
		onto_msg.ID = std::to_string(ID);
		std::cout<< "LOAD ONTOLOGY ID: "<< ID << " Value: " << value << std::endl;
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

