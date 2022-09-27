#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PointStamped.h>
#include <actionlib/client/simple_action_client.h>


// Define a client for to send goal requests to the move base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::PointStamped subscribedCLickedPoint;


void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
 subscribedCLickedPoint.point.x = msg->point.x ;
 subscribedCLickedPoint.point.y = msg->point.y ;
 subscribedCLickedPoint.point.z = msg->point.z ;
}



int main(int argc, char** argv){
	// Initialize the simple navigation goals node
	ros::init(argc, argv, "move_husky");

	ros::NodeHandle n;

	ros::Subscriber clickedPointSubscriber = n.subscribe("/clicked_point", 1000, clickedPointCallback); // subscribe to clicked point

	MoveBaseClient ac("move_base", true);
	move_base_msgs::MoveBaseGoal goal;

	ros::Rate loop_rate(10);
	while(ros::ok()){

		// Wait 5 sec for move base action server to come up
		while(!ac.waitForServer (ros::Duration(5.0))){
			ROS_INFO("Waiting for the move_base action server to come up");
		}

			// set up the frame parameters
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		
		
		
		goal.target_pose.pose.position.x = subscribedCLickedPoint.point.x;
		goal.target_pose.pose.position.y = subscribedCLickedPoint.point.y;
		goal.target_pose.pose.position.z = subscribedCLickedPoint.point.z;
		goal.target_pose.pose.orientation.w = 1.57;

		std::cout<<"Goal X : "<< goal.target_pose.pose.position.x<<std::endl;
		std::cout<<"Goal Y : "<< goal.target_pose.pose.position.y<<std::endl;
		std::cout<<"Goal Z : "<< goal.target_pose.pose.position.z<<std::endl;
		std::cout<<"Goal W : "<< goal.target_pose.pose.orientation.w<<std::endl;

		ROS_INFO ("Sending goal");
		ac.sendGoal (goal);
		// Wait an infinite time for the results
		ac.waitForResult();
		ros::Duration(5.0).sleep();

		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO( "Hooray, reached drop off zone");
		else
			ROS_INFO("The base failed to move forward");

		
		ros::spinOnce(); 

	}

}