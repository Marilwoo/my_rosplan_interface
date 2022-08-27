#include "my_rosplan_interface/moving_action.h"
#include <unistd.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>

namespace KCL_rosplan {
	float x0 = 0.0;
	float y0 = 0.0;
	float x1 = 0.0;
	float y1 = 0.0;
	float x2 = 0.0;
	float y2 = 0.0;
	float x3 = 0.0;
	float y3 = 0.0;
	
	MovingActionInterface::MovingActionInterface(ros::NodeHandle &nh) {
	// here the initialization
	}
	
	bool MovingActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
	// here the implementation of the action
	std::cout << "MOVING ACTION: Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;


	actionlib::SimpleActionClient<erl2::PlanningAction> ac("/reaching_goal", true);
	erl2::PlanningGoal goal;
	ac.waitForServer();
	if(msg->parameters[1].value == "wp0"){
		goal.target_pose.pose.position.x = -2.5;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.w = 0.0;
	}
	else if (msg->parameters[1].value == "wp1"){
		goal.target_pose.pose.position.x = 2.5;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.w = 0.0;
	}
	else if (msg->parameters[1].value == "wp2"){
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = -2.5;
		goal.target_pose.pose.orientation.w = 0.0;
	}
	else if (msg->parameters[1].value == "wp3"){
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = 2.5;
		goal.target_pose.pose.orientation.w = 0.0;
	}
	ac.sendGoal(goal);
	ac.waitForResult();



	ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
	return true;
	}
	
	void marker_position_callback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
	
	int id0 = msg->markers[0].id;
	float x0 = msg->markers[0].pose.position.x;
	float y0 = msg->markers[0].pose.position.y;
	float z0 = msg->markers[0].pose.position.z;
	//printf("Marker %d: x: %f, y: %f, z: %f\n", id0,x0,y0,z0);
	
	int id1 = msg->markers[1].id;
	float x1 = msg->markers[1].pose.position.x;
	float y1 = msg->markers[1].pose.position.y;
	float z1 = msg->markers[1].pose.position.z;
	//printf("Marker %d: x: %f, y: %f, z: %f\n", id1,x1,y1,z1);
	
	int id2 = msg->markers[2].id;
	float x2 = msg->markers[2].pose.position.x;
	float y2 = msg->markers[2].pose.position.y;
	float z2 = msg->markers[2].pose.position.z;
	//printf("Marker %d: x: %f, y: %f, z: %f\n", id2,x2,y2,z2);
	
	int id3 = msg->markers[3].id;
	float x3 = msg->markers[3].pose.position.x;
	float y3 = msg->markers[3].pose.position.y;
	float z3 = msg->markers[3].pose.position.z;
	//printf("Marker %d: x: %f, y: %f, z: %f\n", id3,x3,y3,z3);
	
	//std::cout << "id: " << id << std::endl;
	
	//std::cout<< "size(markers): " << msg->markers.size() << std::endl;
	//std::cout << "qua" << std::endl;	
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "movingaction", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	ros::Subscriber sub = nh.subscribe("/visualization_marker", 10, KCL_rosplan::marker_position_callback);
	KCL_rosplan::MovingActionInterface my_aci(nh);
	my_aci.runActionInterface();
	
	return 0;
}
