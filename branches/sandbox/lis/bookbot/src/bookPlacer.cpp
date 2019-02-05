#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <bookbot/PlacerAction.h>
#include <iostream>

using namespace std;

class PlacerAction {
protected:
	ros::NodeHandle n;
	actionlib::SimpleActionServer<bookbot::PlacerAction> as;
	std::string action_name;
	bookbot::PlacerFeedback feedback;
	bookbot::PlacerResult result;

public:
   PlacerAction(std::string name):
	   as(n, name, boost::bind(&PlacerAction::executeCB, this, _1)), action_name(name) {
   }

   ~PlacerAction(void) {
   }

   void executeCB(const bookbot::PlacerGoalConstPtr &goal) {
		//placing the book action goes here
		ROS_INFO("Working on placing the book...");
		result.result = true;
		as.setSucceeded(result);
	}
};


int main(int argc, char** argv) {
	ros::init(argc, argv, "bookPlacer");
	PlacerAction placer(ros::this_node::getName());
	ros::spin();
}
	
