#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Transform.h>
#include <iostream>
#include "pcl_ros/transforms.h"
#include <tf/tf.h>

class pcl_counter{
    public:
    std::string points_topic;
    ros::Publisher rem;

    pcl_counter(ros::NodeHandle nh){
        
        nh.param<std::string>("points_topic", points_topic, "/lio_sam/mapping/map_global");

        nh.subscribe<sensor_msgs::PointCloud2>("/lio_sam/mapping/map_global", 10, &pcl_counter::countPointsCallback, this);
        
        rem = nh.advertise<sensor_msgs::PointCloud2>("global_map_remapped", 10);
    }

    void countPointsCallback(const sensor_msgs::PointCloud2::ConstPtr & msg){
        sensor_msgs::PointCloud2 out;

        geometry_msgs::Transform t;

        pcl_ros::transformPointCloud("world", t, *msg, out);

        rem.publish(out);
        std::cout <<  "Published" << std::endl;
    }

};


int main(int argc, char **argv){
    ros::init(argc, argv, "count_points");
    ros::NodeHandle nh("~");
    pcl_counter pcl(nh);

    std::cout << "Subscribing to: " << pcl.points_topic << std::endl;

    ros::spin();
}