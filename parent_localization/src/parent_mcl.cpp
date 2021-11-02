#include "parent_mcl/parent_mcl.h"

ParentMCL::ParentMCL() : count(0)
{
    private_nh_.param("obj_topic_name",obj_topic_name_,{"object_positions"});
    private_nh_.param("markers_topic_name",markers_topic_name_,{"markers"});
    private_nh_.param("DISTANCE_OBJ_TH",DISTANCE_OBJ_TH_,{4.0});

    // marker color
    private_nh_.param("trash_can_r",trash_can_.r,{0.0});
    private_nh_.param("trash_can_g",trash_can_.g,{1.0});
    private_nh_.param("trash_can_b",trash_can_.b,{0.0});
    private_nh_.param("fire_hydrant_r",fire_hydrant_.r,{0.0});
    private_nh_.param("fire_hydrant_g",fire_hydrant_.g,{0.0});
    private_nh_.param("fire_hydrant_b",fire_hydrant_.b,{1.0});
    private_nh_.param("bench_r",bench_.r,{1.0});
    private_nh_.param("bench_g",bench_.g,{0.0});
    private_nh_.param("bench_b",bench_.b,{0.0});
    private_nh_.param("big_bench_r",big_bench_.r,{1.0});
    private_nh_.param("big_bench_g",big_bench_.g,{1.0});
    private_nh_.param("big_bench_b",big_bench_.b,{0.0});
    private_nh_.param("fire_extinguisher_r",fire_extinguisher_.r,{0.0});
    private_nh_.param("fire_extinguisher_g",fire_extinguisher_.g,{1.0});
    private_nh_.param("fire_extinguisher_b",fire_extinguisher_.b,{1.0});
    private_nh_.param("kitchenette_icon_r",kitchenette_icon_.r,{1.0});
    private_nh_.param("kitchenette_icon_g",kitchenette_icon_.g,{0.0});
    private_nh_.param("kitchenette_icon_b",kitchenette_icon_.b,{1.0});
    private_nh_.param("toilet_icon_r",toilet_icon_.r,{0.0});
    private_nh_.param("toilet_icon_g",toilet_icon_.g,{0.0});
    private_nh_.param("toilet_icon_b",toilet_icon_.b,{0.0});
    private_nh_.param("chair_r",chair_.r,{1.0});
    private_nh_.param("chair_g",chair_.g,{0.5});
    private_nh_.param("chair_b",chair_.b,{0.5});

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
                marker.color.r = trash_can_.r;
                marker.color.g = trash_can_.g;
                marker.color.b = trash_can_.b;
                marker.color.a = 1.0f;
            }
            else if(msg->object_position[i].Class == "fire_hydrant"){
                marker.color.r = fire_hydrant_.r;
                marker.color.g = fire_hydrant_.g;
                marker.color.b = fire_hydrant_.b;
                marker.color.a = 1.0f;
            }
            else if(msg->object_position[i].Class == "bench"){
                marker.color.r = bench_.r;
                marker.color.g = bench_.g;
                marker.color.b = bench_.b;
                marker.color.a = 1.0f;
            }
            else if(msg->object_position[i].Class == "big_bench"){
                marker.color.r = big_bench_.r;
                marker.color.g = big_bench_.g;
                marker.color.b = big_bench_.b;
                marker.color.a = 1.0f;
            }
            else if(msg->object_position[i].Class == "fire_extinguisher"){
                marker.color.r = fire_extinguisher_.r;
                marker.color.g = fire_extinguisher_.g;
                marker.color.b = fire_extinguisher_.b;
                marker.color.a = 1.0f;
            }
            else if(msg->object_position[i].Class == "kitchenette_icon"){
                marker.color.r = kitchenette_icon_.r;
                marker.color.g = kitchenette_icon_.g;
                marker.color.b = kitchenette_icon_.b;
                marker.color.a = 1.0f;
            }
            else if(msg->object_position[i].Class == "toilet_icon"){
                marker.color.r = toilet_icon_.r;
                marker.color.g = toilet_icon_.b;
                marker.color.b = toilet_icon_.b;
                marker.color.a = 1.0f;
            }
            else if(msg->object_position[i].Class == "chair"){
                marker.color.r = chair_.r;
                marker.color.g = chair_.g;
                marker.color.b = chair_.b;
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