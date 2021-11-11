#include <ros/ros.h>

struct Object
{
	Object(std::string name_,double r_,double g_,double b_) :
	name(name_), r(r_), g(g_), b(b_) {}

    std::string name;
    double r;
    double g;
    double b;
};

class GetObjectName
{
public:
    GetObjectName();
    ~GetObjectName();
    void process();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    XmlRpc::XmlRpcValue object_list_;
    std::vector<Object> objects_;
};

GetObjectName::GetObjectName() : private_nh_("~")
{
    if(!private_nh_.getParam("object_list",object_list_)){
        ROS_ERROR("Error");
    }
    ROS_ASSERT(object_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_INFO("object_list size: %i", (int)object_list_.size());
    for(int i = 0; i < (int)object_list_.size(); i++){
        if(!object_list_[i]["id"].valid() || !object_list_[i]["name"].valid() || !object_list_[i]["r"].valid() || !object_list_[i]["g"].valid() || !object_list_[i]["b"].valid()){
            ROS_WARN("No name");
            return;
        }

        if(object_list_[i]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt){
            int id = static_cast<int>(object_list_[i]["id"]);
        }
        if(object_list_[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString && object_list_[i]["r"].getType() == XmlRpc::XmlRpcValue::TypeDouble && object_list_[i]["g"].getType() == XmlRpc::XmlRpcValue::TypeDouble && object_list_[i]["b"].getType() == XmlRpc::XmlRpcValue::TypeDouble){
            std::string name = static_cast<std::string>(object_list_[i]["name"]);
			double r = static_cast<double>(object_list_[i]["r"]);
			double g = static_cast<double>(object_list_[i]["g"]);
			double b = static_cast<double>(object_list_[i]["b"]);
			Object object(name,r,g,b);
            objects_.push_back(object);
        }
        std::cout << "name: " << objects_[i].name << std::endl;
		std::cout << " r  : " << objects_[i].r << std::endl;
		std::cout << " g  : " << objects_[i].g << std::endl;
		std::cout << " b  : " << objects_[i].b << std::endl;
		std::cout << std::endl;
    }

}

GetObjectName::~GetObjectName() { }

void GetObjectName::process()
{

}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"get_object_name");
    GetObjectName get_object_name;
    //get_object_name.process();
    return 0;
}
