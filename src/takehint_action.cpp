#include <ros/ros.h>

#include "my_rosplan_interface/takehint_action.h"
#include <unistd.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace KCL_rosplan {
	float z0;
	float z1;
	float z2;
	float z3;
	
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
		printf("entrato in wp0\n");
		
		if (z0 == 1.25){
			group.setNamedTarget("1_25_maybe");
			std::cout << "Z0: " << z0 << std::endl;
			}
		else if (z0 == 0.75){
			group.setNamedTarget("0_75_maybe");
			std::cout << "Z0: " << z0 << std::endl;
			}
		else{
			printf("Z0 NON FUNZIONA\n");
			group.setNamedTarget("zero");
			std::cout << "Z0: " << z0 << std::endl;
			}
		group.move(); 
	}
	
	else if (msg->parameters[0].value == "wp1"){
		printf("entrato in wp1\n");
		
		if (z1 == 1.25){
			group.setNamedTarget("1_25_maybe");
			std::cout << "Z1: " << z1 << std::endl;
			}
		else if (z1 == 0.75){
			group.setNamedTarget("0_75_maybe");
			std::cout << "Z1: " << z1 << std::endl;
			}
		else{
			printf("Z1 NON FUNZIONA\n");
			group.setNamedTarget("zero");
			std::cout << "Z1: " << z1 << std::endl;
			}
		group.move(); 
	}
	else if (msg->parameters[0].value == "wp2"){
		printf("entrato in wp2\n");
		
		if (z2 == 1.25){
			group.setNamedTarget("1_25_maybe");
			std::cout << "Z2: " << z2 << std::endl;
			}
		else if (z2 == 0.75){
			group.setNamedTarget("0_75_maybe");
			std::cout << "Z2: " << z2 << std::endl;
			}
		else{
			printf("Z2 NON FUNZIONA\n");
			group.setNamedTarget("zero");
			std::cout << "Z2: " << z2 << std::endl;
			}
		group.move(); 
	}
	else if (msg->parameters[0].value == "wp3"){
		printf("entrato in wp3\n");
		
		if (z3 == 1.25){
			group.setNamedTarget("1_25_maybe");
			std::cout << "Z3: " << z3 << std::endl;
			}
		else if (z3 == 0.75){
			group.setNamedTarget("0_75_maybe");
			std::cout << "Z3: " << z3 << std::endl;
			}
		else{
			printf("Z3 NON FUNZIONA\n");
			group.setNamedTarget("zero");
			std::cout << "Z3: " << z3 << std::endl;
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
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "takehintaction", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	ros::NodeHandle nh1;
	ros::Subscriber sub = nh1.subscribe("/visualization_marker", 10, KCL_rosplan::marker_position_callback);
	KCL_rosplan::TakeHintInterface my_aci(nh);
	my_aci.runActionInterface();
	
	//Moveit
	
  	
  	
	
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	return 0;
}

