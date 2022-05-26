#include <iostream>
#include <ros/ros.h>
#include <quad.h>

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "quad_odom");
    ros::NodeHandle nh("~");
    quad_class quad_class(nh);
    ros::spin();
    return 0;

}