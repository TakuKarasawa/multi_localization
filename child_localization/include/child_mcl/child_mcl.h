#ifndef CHILD_MCL_H_
#define CHILD_MCL_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <random>
#include <sstream>
#include <fstream>

#include <Eigen/Dense>

#include "object_detector_msgs/ObjectPosition.h"
#include "object_detector_msgs/ObjectPositions.h"

class MultivariateNormal
{
public:

    MultivariateNormal(Eigen::Vector2d mean,Eigen::Matrix2d cov) :
        mean_(mean), cov_(cov) {}

    double pdf(Eigen::Vector2d x)
    {
        return 1.0/(double)std::sqrt(std::pow(2*M_PI,2)*cov_.determinant())*std::exp(-0.5*(x - mean_).transpose()*cov_.inverse()*(x - mean_));
    }

private:
    Eigen::Vector2d mean_;
    Eigen::Matrix2d cov_;

};

struct ObjectError
{
	ObjectError(int id_,double error_) :
		id(id_), error(error_) {}
	
	std::string name;
	int id;
	double error;
};


struct ObjectDistance
{
	ObjectDistance(std::string name_,double distance_,double theta_) :
		name(name_), distance(distance_), theta(theta_) {}

	void push_object_error(ObjectError object_error)
	{ 
		obejct_errors.push_back(object_error); 
	}

	void sort_object_error()
	{
		std::sort(obejct_errors.begin(),obejct_errors.end(),[](const ObjectError& a,const ObjectError& b) { return a.error < b.error; });
	}

	std::string name;
	double distance;
    double theta;

	std::vector<ObjectError> obejct_errors;
};

struct ObjectNode
{
	ObjectNode(std::string name_,float r_,float g_,float b_) :
	name(name_), r(r_), g(g_), b(b_) {}

	std::string name;
	float r;
	float g;
	float b;
};

class ChildMCL
{
public:
    ChildMCL();
    ~ChildMCL();
    void process();

private:
    class Particle
    {
    public:
        Particle(ChildMCL* child_mcl);
        ~Particle();

        void init(double x,double y,double yaw,double x_cov,double y_cov,double yaw_cov);
        void motion_update();
        void measurement_update();
        void state_transition(double dx,double dy,double dyaw);

        geometry_msgs::PoseStamped pose_;
        double likelihood_;

    private:
        ChildMCL* child_mcl_;
    };

    void map_callback(const nav_msgs::OccupancyGridConstPtr& msg);
    void obj_poses_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg);

    void load_parameter();
    void read_csv();
    std::vector<std::string> split(std::string& input,char delimiter);

    void initialize();
    void load_odometry(geometry_msgs::PoseStamped& odometry);
    void set_pose(geometry_msgs::PoseStamped& pose,double x,double y,double yaw);
    void spread_particles();
    void motion_update();
    void measurement_update();
    void calc_likelihood();
    void resampling();
    void calc_estimated_pose();
    void calc_cov();
    void publish_estimated_pose();
    void publish_tf();
    
    int map_grid_data(double x,double y);
    double calc_yaw_from_msg(geometry_msgs::Quaternion q);
    double calc_angle_diff(double a,double b);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber map_sub_;
    ros::Subscriber obj_poses_sub_;
    ros::Publisher estimated_pose_pub_;
    ros::Publisher estimated_poses_pub_;

    nav_msgs::OccupancyGrid map_;
    geometry_msgs::PoseStamped estimated_pose_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped previous_pose_;
    geometry_msgs::PoseArray estimated_poses_;
    object_detector_msgs::ObjectPositions obj_poses_;
    visualization_msgs::MarkerArray markers_;

    std::vector<ObjectNode> objects_;
    std::vector<Particle> particles_;
    std::random_device seed_;
    std::mt19937 engine_;
    XmlRpc::XmlRpcValue object_list_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;

    // parameter
    bool has_received_map_;
    bool has_received_obj_poses_;
    bool is_update_;
    int count_;
    int max_index_;
    double x_cov_;
    double y_cov_;
    double yaw_cov_;
    double likelihood_slow_;
    double likelihood_fast_;
    double likelihood_sum_;
    double likelihood_average_;
    double likelihood_max_;
    double distance_sum_;
    double angle_sum_;


    // load parameter
    std::string dir_path_ = "/home/amsl/catkin_ws/src/multi_localization/child_localization/record/";
    std::string file_name_;
    std::string obj_poses_topic_name_;
    std::string estimated_pose_topic_name_;
    std::string estimated_poses_topic_name_;
    std::string map_topic_name_;
    std::string map_frame_id_;
    std::string odom_frame_id_;
    std::string base_link_frame_id_;
    double HZ_;
    int NUM_OF_PARTICLES_;
    double INIT_X_;
    double INIT_Y_;
    double INIT_YAW_;
    double INIT_X_COV_;
    double INIT_Y_COV_;
    double INIT_YAW_COV_;
    double MAX_RANGE_;
    int RANGE_STEP_;
    double X_COV_TH_;
    double Y_COV_TH_;
    double YAW_COV_TH_;
    double ALPHA_1_;
    double ALPHA_2_;
    double ALPHA_3_;
    double ALPHA_4_;
    double ALPHA_SLOW_;
    double ALPHA_FAST_;
    double HIT_COV_;
    double LAMBDA_SHORT_;
    double Z_HIT_;
    double Z_SHORT_;
    double Z_MAX_;
    double Z_RAND_;
    double DISTANCE_TH_;
    double ANGLE_TH_;
    double SELECTION_RATE_;

    double DISTANCE_DEV_RATE_;
    double DIRECTION_DEV_;

};

#endif  // CHILD_EKF_H_