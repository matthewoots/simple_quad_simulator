#include "ros/ros.h"

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>

#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_publisher");
    ros::NodeHandle nh("~");

    double publish_time = 10.0;
    double publish_rate = 3.0;

    std::string _map_path;
    nh.param<std::string>("map/path", _map_path, "");
    printf("[map_publisher] pcd path: %s\n", _map_path.c_str());

    // try to load the file
    pcl::PointCloud<pcl::PointXYZ> cloud_map;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(
        _map_path, cloud_map) == -1) 
    {
        printf("[map_publisher] %sno valid pcd used%s!\n", KRED, KNRM);
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud_map, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "world";

    ros::Publisher map_pub = 
        nh.advertise<sensor_msgs::PointCloud2>("/global_map", 3);

    ros::Rate loop_rate(publish_rate);
    ros::Time begin = ros::Time::now();

    while (ros::ok() && (ros::Time::now() - begin).toSec() < publish_time)
    {
        map_pub.publish(cloud_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}