#include "map_update_checker/map_update_checker.h"

MapUpdateChecker::MapUpdateChecker() :
	private_nh_("~"), 
	count_(0), is_first_(true), first_time_(0.0),
	has_received_pose_(false), has_received_obj_(false),
    RANGE_TH_(4.00)
{
	private_nh_.param("file_name",file_name_,{"arrange_1.csv "});
	private_nh_.param("marker_frame_id",marker_frame_id_,{"map"});
    private_nh_.param("est_pose_topic_name",est_pose_topic_name_,{"/est_pose"});
    private_nh_.param("obj_topic_name",obj_topic_name_,{"/object_positions"});

	markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/markers",1);

    pose_sub_ = nh_.subscribe(est_pose_topic_name_,1,&MapUpdateChecker::pose_callback,this);
    obj_sub_ = nh_.subscribe(obj_topic_name_,1,&MapUpdateChecker::obj_callback,this);

	load_parameter();
}

MapUpdateChecker::~MapUpdateChecker() { }

void MapUpdateChecker::pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	pose_ = *msg;
	has_received_pose_ = true;
}

void MapUpdateChecker::obj_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg)
{
	obj_poses_ = *msg;
	has_received_obj_ = true;
}

std::vector<std::string> MapUpdateChecker::split(std::string& input,char delimiter)
{
	std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while(std::getline(stream,field,delimiter)) result.push_back(field);
    
    return result;
}

void MapUpdateChecker::load_parameter()
{
    if(!private_nh_.getParam("object_list",object_list_)){
        ROS_WARN("Could not load objects list");
        return;
    }
    ROS_ASSERT(object_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i = 0; i < (int)object_list_.size(); i++){
        if(!object_list_[i]["name"].valid() || !object_list_[i]["id"].valid() || !object_list_[i]["r"].valid() || !object_list_[i]["g"].valid() || !object_list_[i]["b"].valid()){
            ROS_WARN("object_list is valid");
            return;
        }
        if(object_list_[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString && object_list_[i]["r"].getType() == XmlRpc::XmlRpcValue::TypeDouble && object_list_[i]["g"].getType() == XmlRpc::XmlRpcValue::TypeDouble && object_list_[i]["b"].getType() == XmlRpc::XmlRpcValue::TypeDouble){
            std::string name = static_cast<std::string>(object_list_[i]["name"]);
            double r = static_cast<double>(object_list_[i]["r"]);
            double g = static_cast<double>(object_list_[i]["g"]);
            double b = static_cast<double>(object_list_[i]["b"]);
            ObjectNode object_node(name,(float)r,(float)g,(float)b);
            objects_.push_back(object_node);
        }
    }
}

void MapUpdateChecker::read_csv(visualization_msgs::MarkerArray& markers)
{
	std::ifstream ifs_csv_file(dir_path_ + file_name_);
    std::string line;
    while(std::getline(ifs_csv_file,line)){
        std::vector<std::string> strvec = split(line,',');

        if(is_first_){
            first_time_ = std::stod(strvec.at(0));
            is_first_ = false;
        }

        visualization_msgs::Marker marker;
        marker.header.frame_id = marker_frame_id_;
        marker.ns = strvec.at(1);
        marker.id = count_;

        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration();

        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.6;

        marker.pose.position.x = std::stod(strvec.at(2));
        marker.pose.position.y = std::stod(strvec.at(3));
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        bool is_color = false;
        for(const auto&object : objects_){
            if(strvec.at(1) == object.name){
                marker.color.r = object.r;
                marker.color.g = object.g;
                marker.color.b = object.b;
                marker.color.a = 1.0f;
                is_color = true;
            }
            if(is_color) break;
        }

        if(!is_color){
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;
        }
        
        markers.markers.push_back(marker);
        count_ ++;
    }
}

void MapUpdateChecker::check_object_range(visualization_msgs::MarkerArray& markers)
{
    std::vector<ObjectDistance> distances;

    for(auto&o : obj_poses_.object_position){
        if(o.Class == "roomba") continue;
        if(o.Class == "elevator") continue;
        if(o.Class == "table") continue;

        double distance = std::sqrt(o.x*o.x + o.z*o.z);
        std::cout << "distance: " << distance << std::endl;

        if(distance > RANGE_TH_) continue;

        ObjectDistance object_distance(o.Class,distance);
        distances.push_back(object_distance);
    }
    std::cout << std::endl;


    // debug
    std::cout << "distances_size: " << distances.size() << std::endl;
    if(distances.size() == 0) return;
    for(auto&d : distances){
        std::cout << "name: " << d.name << std::endl;
    }
    
    /*
    for(auto&d : distances){
        std::cout << "class: " << d.name << std::endl;
        std::cout << "distance: " << d.distance << std::endl;
    }
    */

    std::cout << "sort" << std::endl;
    for(auto&d : distances){
        for(auto&m : markers.markers){
            if(d.name == m.ns){
                double distance = std::sqrt(std::pow(m.pose.position.x - pose_.pose.position.x,2) + std::pow(m.pose.position.y - pose_.pose.position.y,2));
                double error = std::sqrt(std::pow(distance - d.distance,2));
                ObjectError object_error(m.id,error);
                d.push_object_error(object_error);
            }
        }
        d.sort_object_error();
    }

    for(auto&d : distances){
        for(auto&e : d.obejct_errors){
            std::cout << "id: " << e.id << ", error: " << e.error << std::endl;
        }
        std::cout << std::endl;
    }

    for(auto&d : distances){
        std::cout << "detect_id: " << d.obejct_errors[0].id << std::endl;
        std::cout << std::endl;
        for(auto&m : markers.markers){
            if(d.obejct_errors[0].id == m.id){
                if(d.distance <= RANGE_TH_ && d.obejct_errors[0].error < 1){
                    m.color.a = 1.0f;
                    d.obejct_errors.erase(d.obejct_errors.begin());
                    //break;
                }
            }
        }
    }

}

void MapUpdateChecker::process()
{
	visualization_msgs::MarkerArray markers;
	read_csv(markers);
	ros::Rate rate(1);
	while(ros::ok()){
		markers_pub_.publish(markers);

		if(has_received_pose_ && has_received_obj_){
            check_object_range(markers);
			has_received_pose_ = false;
            has_received_obj_ = false;
		}

		ros::spinOnce();
		rate.sleep();
	}	
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"map_update_checker");
	MapUpdateChecker map_update_checker;
	map_update_checker.process();
	return 0;
}