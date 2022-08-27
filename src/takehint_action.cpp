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
	
		

	if(msg->parameters[0].value == "wp0"){
		printf("entrato in wp0\n");
		
		robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	  	const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
		ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
	  	
	  	
		moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
	  	kinematic_state->setToDefaultValues();
	 	const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
	 	moveit::planning_interface::MoveGroupInterface group("arm");
	 	const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  
  
		geometry_msgs::Pose pose1;
		//moveit::planning_interface::MoveGroupInterface group("arm");
	
  		pose1.orientation.w = 0.00;
  		pose1.orientation.x = 0.00;
		pose1.orientation.y = 0.00;
		pose1.orientation.z = 0.00;
		pose1.position.x =  2.50;
		pose1.position.y =  -2.00;
		pose1.position.z =  1.25;
		std::cout << "HINT HEIGHT Z0: "<< pose1.position.z << std::endl;
		
  		group.setStartStateToCurrentState();
		group.setApproximateJointValueTarget(pose1, "cluedo_link");
		std::vector<double> joint_values;
		double timeout = 0.1;
		bool found_ik = kinematic_state->setFromIK(joint_model_group, pose1, timeout);
		
		//if (found_ik)
	 	//{
			kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
			//for (std::size_t i = 0; i < joint_names.size(); ++i)
			//{
			  //ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
			//}
		 //}
		 //else
	    //{
			//ROS_INFO("Did not find IK solution");
	    //}
		  
		group.setJointValueTarget(joint_values);
		group.setStartStateToCurrentState();
		group.setGoalOrientationTolerance(0.01);
		group.setGoalPositionTolerance(0.01);
		 
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		group.plan(my_plan); 
		group.execute(my_plan);
	}
	
	else if (msg->parameters[1].value == "wp1"){
		
	}
	else if (msg->parameters[1].value == "wp2"){
		
	}
	else if (msg->parameters[1].value == "wp3"){

	}
	//ac.sendGoal(goal);
	//ac.waitForResult();

	ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
	return true;
	}
	
	
	void marker_position_callback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
	
	//int id0 = msg->markers[0].id;
	float z0 = msg->markers[0].pose.position.z;
	//printf("Marker %d: x: %f, y: %f, z: %f\n", id0,x0,y0,z0);
	
	//int id1 = msg->markers[1].id;
	float z1 = msg->markers[1].pose.position.z;
	//printf("Marker %d: x: %f, y: %f, z: %f\n", id1,x1,y1,z1);
	
	//int id2 = msg->markers[2].id;
	float z2 = msg->markers[2].pose.position.z;
	//printf("Marker %d: x: %f, y: %f, z: %f\n", id2,x2,y2,z2);
	
	//int id3 = msg->markers[3].id;
	float z3 = msg->markers[3].pose.position.z;
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

