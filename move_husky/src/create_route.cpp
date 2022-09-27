#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PointStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <limits>
#include <std_msgs/Int32.h>
#include "geometry_msgs/Point.h"
#include "move_husky/my_msg.h"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std_msgs::Int32 userPointCount;
move_husky::my_msg userSelectVec;
geometry_msgs::PointStamped subscribedCLickedPoint;
visualization_msgs::Marker  leaveMarker;

float counter;
int user_value;

void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg){ 
    subscribedCLickedPoint.point.x= msg->point.x ;
    subscribedCLickedPoint.point.y= msg->point.y ;
    subscribedCLickedPoint.point.z= msg->point.z ;
    counter++;
}

int main (int argc, char** argv){
    ros::init(argc, argv, "move_husky");

    ros::NodeHandle n;

    ros::Subscriber clickedPointSubscriber = n.subscribe("/clicked_point", 1000, clickedPointCallback); // subscribe to clicked point
    ros::Publisher  pointPublisher=n.advertise<std_msgs::Int32>("/user_selected_stop",1);
    ros::Publisher  vectorPublisher=n.advertise<move_husky::my_msg>("my_topic",1);
    ros::Publisher  markerPub=n.advertise<visualization_msgs::Marker>("visualization_marker",0);
    
    
    MoveBaseClient ac("move_base", true);
    

    move_base_msgs::MoveBaseGoal goal;
    
    std::cout<<"Please enter how many stops you will choose:";
    std::cin>>user_value;
    userPointCount.data = user_value;

    ros::Rate loop_rate(10);  
    while(ros::ok()){
        
        geometry_msgs::Point str_point;
        
        str_point.x=subscribedCLickedPoint.point.x;
        str_point.y=subscribedCLickedPoint.point.y;
        str_point.z=subscribedCLickedPoint.point.z;
    
        auto itPos = userSelectVec.points.begin()+counter;
        auto newIt = userSelectVec.points.insert(itPos,str_point);
        userSelectVec.another_field=counter;


        if(counter<=user_value){
            leaveMarker.header.frame_id="map";
            leaveMarker.header.stamp=ros::Time::now();
            leaveMarker.ns="sphere";
            leaveMarker.id=counter;
            leaveMarker.type=visualization_msgs::Marker::SPHERE;
            leaveMarker.action=visualization_msgs::Marker::ADD;

            leaveMarker.pose.position.x=str_point.x;
            leaveMarker.pose.position.y=str_point.y;
            leaveMarker.pose.position.z=str_point.z;  
            leaveMarker.pose.orientation.x = 0.0;                             
            leaveMarker.pose.orientation.y = 0.0;
            leaveMarker.pose.orientation.z = 0.0;
            leaveMarker.pose.orientation.w = 1.0;
            leaveMarker.scale.x = 0.3;
            leaveMarker.scale.y = 0.3;
            leaveMarker.scale.z = 0.3;
            leaveMarker.color.a = 1.0;
            leaveMarker.color.r = 1.0;
            leaveMarker.color.g = 0.0;
            leaveMarker.color.b = 0.0;
            
            leaveMarker.mesh_resource="package://pr2_description/meshes/base_v0/base.dae";
            markerPub.publish(leaveMarker);
        }
                
        if(user_value==counter){      
            vectorPublisher.publish(userSelectVec);
            pointPublisher.publish(userPointCount);
        }
        ros::spinOnce();

        loop_rate.sleep();
        
    }   
   }