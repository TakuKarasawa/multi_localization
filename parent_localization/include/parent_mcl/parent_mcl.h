#ifndef PARENT_MCL_H_
#define PARENT_MCL_H_

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <fstream>

#include "mcl/mcl.h"
#include "object_detector_msgs/ObjectPosition.h"
#include "object_detector_msgs/ObjectPositions.h"

struct MarkerColor
{
    float r;
    float g;
    float b;
};

class ParentMCL : public MCL
{
public:
    ParentMCL();
    ~ParentMCL();
    
private:
    void obj_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg);

    ros::Subscriber obj_sub_;
    ros::Publisher markers_pub_;

    // marker color
    MarkerColor trash_can_;
    MarkerColor fire_hydrant_;
    MarkerColor bench_;
    MarkerColor big_bench_;
    MarkerColor fire_extinguisher_;
    MarkerColor kitchenette_icon_; 
    MarkerColor toilet_icon_;
    MarkerColor chair_; 

    // parameter
    std::string obj_topic_name_;
    std::string markers_topic_name_;

    int count;
    double DISTANCE_OBJ_TH_;
};

#endif  // PARENT_MCL_H_