#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class MarkerStore
{
public:
    MarkerStore();
    void process() { ros::spin(); }

private:
    void markers_callback(const visualization_msgs::MarkerArrayConstPtr& msg);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;


};
