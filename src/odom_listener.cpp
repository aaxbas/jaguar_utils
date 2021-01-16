#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iostream>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "subber");

    ros::NodeHandle nh;

    tf::TransformListener listener;

    ros::Rate rate(10.0);

    while (nh.ok())
    {
        tf::StampedTransform transform;

        listener.waitForTransform("/odom","/base_link",ros::Time::now(), ros::Duration(0.2));
        try
        {
            listener.lookupTransform("/odom", "/base_link",
                                     ros::Time(0), transform);
            std::cout << "Found tf at: " << transform.stamp_ << std::endl; 
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        rate.sleep();
    }
    return 0;
}