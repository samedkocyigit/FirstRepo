#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker dropMarker;
int counter=100;

struct Position{
    float pose_position_x,pose_position_y,pose_position_z;
    float pose_oriantation_x,pose_oriantation_y,pose_oriantation_z,pose_oriantation_w;
}points;

void createPathCallback(const nav_msgs::Path::ConstPtr& msg){
    points.pose_position_x=msg->poses.data()->pose.position.x;
    points.pose_position_y=msg->poses.data()->pose.position.y;
    points.pose_position_z=msg->poses.data()->pose.position.z;
    points.pose_oriantation_x=msg->poses.data()->pose.orientation.x;
    points.pose_oriantation_y=msg->poses.data()->pose.orientation.y;
    points.pose_oriantation_z=msg->poses.data()->pose.orientation.z;
    points.pose_oriantation_w=msg->poses.data()->pose.orientation.w;
    counter++;
}

int main(int argc, char** argv){
    ros::init(argc,argv,"create_path");

    ros::NodeHandle n;
    
    ros::Subscriber create_path       =n.subscribe("/move_base/DWAPlannerROS/local_plan",1000,createPathCallback);
    ros::Publisher  markerPublisher   =n.advertise<visualization_msgs::Marker>("/visualization_marker",1000);
    
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        dropMarker.header.frame_id="odom";
        dropMarker.header.stamp=ros::Time::now();
        dropMarker.ns="sphere";
        dropMarker.id=counter;
        dropMarker.type=visualization_msgs::Marker::SPHERE;

        dropMarker.action=visualization_msgs::Marker::ADD;
    
        dropMarker.pose.position.x=points.pose_position_x;
        dropMarker.pose.position.y=points.pose_position_y;
        dropMarker.pose.position.z=points.pose_position_z;                   

        dropMarker.pose.orientation.x = points.pose_oriantation_x; 
        dropMarker.pose.orientation.y = points.pose_oriantation_y; 
        dropMarker.pose.orientation.z = points.pose_oriantation_z; 
        dropMarker.pose.orientation.w = points.pose_oriantation_w; 
        dropMarker.scale.x = 0.1;
        dropMarker.scale.y = 0.1;
        dropMarker.scale.z = 0.1;
        dropMarker.color.a = 1.0;
        dropMarker.color.r = 0.0;
        dropMarker.color.g = 1.0;
        dropMarker.color.b = 0.0;
                                
        dropMarker.mesh_resource="package://pr2_description/meshes/base_v0/base.dae";
        markerPublisher.publish(dropMarker); 

        ros::spinOnce();
        loop_rate.sleep();
    }
}