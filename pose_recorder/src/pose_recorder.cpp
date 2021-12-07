#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sstream>
#include <fstream>

class PoseRecorder
{
public:
	PoseRecorder() :
		private_nh_("~"),
		is_first_(true), has_received_odom_(false), has_received_mcl_pose_(false)
	{
		private_nh_.param("odom_topic_name",odom_topic_name_,{"/roomba/odometry"});
		private_nh_.param("mcl_pose_topic_name",mcl_pose_topic_name_,{"/est_pose"});

		private_nh_.param("HZ",HZ_,{10});

		odom_sub_ = nh_.subscribe(odom_topic_name_,HZ_,&PoseRecorder::odom_callback,this);
		mcl_pose_sub_ = nh_.subscribe(mcl_pose_topic_name_,HZ_,&PoseRecorder::mcl_pose_callback,this);
	}

	void process()
	{
		ros::Rate rate(HZ_);
		while(ros::ok()){
			if(is_first_){
				start_time_ = ros::Time::now();
				is_first_ = false;
			}

			static std::ofstream ofs("/home/amsl/catkin_ws/src//multi_localization/pose_recorder/record/record_7.csv");
			ofs << ros::Time::now().toSec() - start_time_.toSec() << ","
			    << odom_.pose.pose.position.x << "," << odom_.pose.pose.position.y << "," << calc_yaw_from_msg(odom_.pose.pose.orientation) << ","
			    << mcl_pose_.pose.position.x << "," << mcl_pose_.pose.position.y << calc_yaw_from_msg(mcl_pose_.pose.orientation) << std::endl;

			ros::spinOnce();
			rate.sleep();
		}
	}

private:
	void odom_callback(const nav_msgs::OdometryConstPtr& msg)
	{
		odom_ = *msg;
		has_received_odom_ = true;
	}

	void mcl_pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
	{
		mcl_pose_ = *msg;
		has_received_mcl_pose_ = true;
	}

    double calc_yaw_from_msg(geometry_msgs::Quaternion q)
    {
        double roll, pitch, yaw;
        tf2::Quaternion tf_q(q.x,q.y,q.z,q.w);
        tf2::Matrix3x3(tf_q).getRPY(roll,pitch,yaw);

        return yaw;
    }

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	ros::Subscriber odom_sub_;
	ros::Subscriber mcl_pose_sub_;

	nav_msgs::Odometry odom_;
	geometry_msgs::PoseStamped mcl_pose_;

	bool is_first_;
	bool has_received_odom_;
	bool has_received_mcl_pose_;
	ros::Time start_time_;

	std::string odom_topic_name_;
	std::string mcl_pose_topic_name_;

	double HZ_;
};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"pose_recorder");
	PoseRecorder pose_recorder;
	pose_recorder.process();
	return 0;
}
