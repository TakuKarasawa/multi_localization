#include "child_ekf/child_ekf.h"

ChildEKF::ChildEKF() :
    private_nh_("~"),
    tf_listener_(tf_buffer_)
{

}

void ChildEKF::odometry_callbak(const nav_msgs::OdometryConstPtr& msg)
{

}

void ChildEKF::object_positions_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg)
{

}

void ChildEKF::timer_callback(const ros::TimerEvent& event)
{


}


void ChildEKF::process()
{

}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"child_ekf");
    return 0;
}
