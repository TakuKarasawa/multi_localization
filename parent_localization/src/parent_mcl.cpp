#include "parent_mcl/parent_mcl.h"

ParentMCL::ParentMCL() : count(0)
{
    private_nh_.param("obj_topic_name",obj_topic_name_,{"object_positions"});
    private_nh_.param("markers_topic_name",markers_topic_name_,{"markers"});
    private_nh_.param("DISTANCE_OBJ_TH",DISTANCE_OBJ_TH_,{4.0});

    obj_sub_ = nh_.subscribe(obj_topic_name_,1,&ParentMCL::obj_callback,this);
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(markers_topic_name_,1);
}

ParentMCL::~ParentMCL() {}

void ParentMCL::obj_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg)
{   
    visualization_msgs::MarkerArray markers;
    for(int i = 0; i < msg->object_position.size(); i++){
        double distance = std::sqrt(std::pow(msg->object_position[i].x,2) + std::pow(msg->object_position[i].z,2));
        double theta = std::atan2(msg->object_position[i].z,msg->object_position[i].x) - 0.5*M_PI;

        if(distance < DISTANCE_OBJ_TH_){
            visualization_msgs::Marker marker;
            marker.header.frame_id = map_frame_id_;
            marker.header.stamp = ros::Time::now();
            marker.ns = msg->object_position[i].Class;
            marker.id = count;

            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration();

            marker.scale.x = 0.4;
            marker.scale.y = 0.4;
            marker.scale.z = 0.6;
            marker.pose.position.x = est_pose_.pose.position.x + distance*std::cos(calc_yaw_from_quat(est_pose_.pose.orientation) + theta);
            marker.pose.position.y = est_pose_.pose.position.y + distance*std::sin(calc_yaw_from_quat(est_pose_.pose.orientation) + theta);
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.y = 1.0;

            if(msg->object_position[i].Class == "trash_can"){
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0f;
            }
            else if(msg->object_position[i].Class == "fire_hydrant"){
                marker.color.r = 0.0f;
                marker.color.g = 0.0f;
                marker.color.b = 1.0f;
                marker.color.a = 1.0f;
            }
            else if(msg->object_position[i].Class == "bench"){
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0f;
            }
            else if(msg->object_position[i].Class == "big_bench"){
                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0f;
            }
            else if(msg->object_position[i].Class == "fire_extinguisher"){
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 1.0f;
                marker.color.a = 1.0f;
            }
            else if(msg->object_position[i].Class == "kitchenette_icon"){
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 1.0f;
                marker.color.a = 1.0f;
            }
            else if(msg->object_position[i].Class == "toilet_icon"){
                marker.color.r = 0.0f;
                marker.color.g = 0.5f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0f;
            }
            else if(msg->object_position[i].Class == "chair"){
                marker.color.r = 1.0f;
                marker.color.g = 0.5f;
                marker.color.b = 0.5f;
                marker.color.a = 1.0f;
            }
            else{
                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 1.0f;
                marker.color.a = 1.0f;
            }
            markers.markers.push_back(marker);

            static std::ofstream ofs("/home/amsl/catkin_ws/src/multi_localization/parent_localization/markers.csv");
            ofs << marker.header.stamp << "," << marker.ns << "," << marker.pose.position.x << "," << marker.pose.position.y << std::endl;
            count ++;
        }
    }
    markers_pub_.publish(markers);
}