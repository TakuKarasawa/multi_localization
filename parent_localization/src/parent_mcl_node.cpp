#include "parent_mcl/parent_mcl.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"parent_mcl");
    ParentMCL mcl;
    mcl.process();
    return 0;
}
