#ifndef MCL_H_
#define MCL_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

#include <vector>
#include <random>

class MCL
{
public:
    MCL();
    ~MCL();
    void process();

protected:
    class Particle
    {
    public:
        Particle(MCL* mcl);
        ~Particle();

        void init(double x,double y,double yaw,double x_cov,double y_cov,double yaw_cov);
        void motion_update();
        void measurement_update();
        void move(double dx,double dy,double dyaw);

        geometry_msgs::PoseStamped pose_;
        double likelihood_;
    private:
        MCL* m_;
    };

    void map_callback(const nav_msgs::OccupancyGridConstPtr& msg);
    void scan_callback(const sensor_msgs::LaserScanConstPtr& msg);
    void timer_callback(const ros::TimerEvent& event);

    void set_pose(geometry_msgs::PoseStamped& pose,double x,double y,double yaw);
    void spread_particles();
    void motion_process();
    void measurement_process();
    void calc_likelihood();
    void resampling();
    void calc_est_pose();
    void calc_cov();
    void publish_est_pose();
    void publish_tf();

    int map_grid_data(double x,double y);
    double calc_yaw_from_quat(geometry_msgs::Quaternion q);
    double calc_angle_diff(double a,double b);
    double calc_range(double x,double y,double yaw);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Timer timer_;
    ros::Subscriber scan_sub_;
    ros::Subscriber map_sub_;
    ros::Publisher est_pose_pub_;
    ros::Publisher est_poses_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;

    geometry_msgs::PoseArray est_poses_;
    geometry_msgs::PoseStamped est_pose_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped previous_pose_;
    nav_msgs::OccupancyGrid map_;
    sensor_msgs::LaserScan scan_;

    std::vector<Particle> particles_;
    std::random_device seed_;
    std::mt19937 engine_;


    // parameter
    std::string map_topic_name_;
    std::string scan_topic_name_;
    std::string est_pose_topic_name_;
    std::string est_poses_topic_name_;
    std::string map_frame_id_;
    std::string odom_frame_id_;
    std::string base_link_frame_id_;
    bool has_received_map_;
    bool is_update_;
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
    int max_index_;

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

};

#endif // MCL_H_
