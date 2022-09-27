#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PointStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int32.h>
#include <vector>
#include "geometry_msgs/Point.h"
#include "move_husky/my_msg.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std_msgs::Int32 userPoint;
move_husky::my_msg subVector;

bool flag;
int counter;

void userSelectVectorCallback(const  move_husky::my_msg::ConstPtr& msg){
    subVector.another_field=msg->another_field;
    for(int i=0;i<=subVector.another_field;i++){
        geometry_msgs::Point points_vector;
        points_vector.x=msg->points[i].x;
        points_vector.y=msg->points[i].y;
        points_vector.z=msg->points[i].z; 
        auto itPos=subVector.points.begin()+i;
        auto newIt=subVector.points.insert(itPos,points_vector);
    } 
}

void userSelectedNumCallback(const std_msgs::Int32::ConstPtr& msg){ 
    userPoint.data = msg->data;
    flag=true;
}

int main (int argc, char** argv){
    ros::init(argc, argv, "move_husky_ring");

    ros::NodeHandle n;

    ros::Subscriber user_point      = n.subscribe("/user_selected_stop", 1000, userSelectedNumCallback); // subscribe to clicked point
    ros::Subscriber user_vector     = n.subscribe("my_topic",1,userSelectVectorCallback);

    MoveBaseClient ac("move_base", true);

    move_base_msgs::MoveBaseGoal goal;

    
    ros::Rate loop_rate(10);
    while(ros::ok()){
        
        if(flag==true){
            ros::Rate loop_rate(10);
            while(ros::ok()){
                

                while(!ac.waitForServer (ros::Duration(5.0))){
                    ROS_INFO("Waiting for the move_base action server to come up");
                }
            
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
             
                if(subVector.another_field==userPoint.data){
                    for(int j=0;j<2;j++){
                        for(int k=1;k<=subVector.another_field; k++){
                            goal.target_pose.pose.position.x = subVector.points[k].x;
                            goal.target_pose.pose.position.y = subVector.points[k].y;
                            goal.target_pose.pose.position.z = subVector.points[k].z;
                            goal.target_pose.pose.orientation.w = 1.57;

                            std::cout<<"Goal X : "<< goal.target_pose.pose.position.x<<std::endl;
                            std::cout<<"Goal Y : "<< goal.target_pose.pose.position.y<<std::endl;
                            std::cout<<"Goal Z : "<< goal.target_pose.pose.position.z<<std::endl;
                            std::cout<<"Goal W : "<< goal.target_pose.pose.orientation.w<<std::endl;

                            ROS_INFO ("Sending goal");
                            ac.sendGoal (goal);
                            
                            
                            ac.waitForResult();
                            ros::Duration(5.0).sleep();

                            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                                ROS_INFO( "Hooray, reached drop off zone");
                            else
                                ROS_INFO("The base failed to move forward");
                        }
                    }
                flag=false;
                ros::shutdown(); //process ends when the round is completed
                }
            ros::spinOnce();
            loop_rate.sleep();
            }   
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}    