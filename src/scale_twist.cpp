#include <ros/ros.h>
#include <nav_msgs/Odometry.h>


ros::Publisher odom_pub;

int gain;

void odomCallback(const nav_msgs::OdometryConstPtr& msg){
    nav_msgs::Odometry odom = *msg;

    odom.twist.twist.linear.x = odom.twist.twist.linear.x*gain;
    odom.twist.twist.angular.z =  odom.twist.twist.angular.z*gain;

    odom_pub.publish(odom);
}


int main(int argc, char **argv){

    ros::init(argc, argv, "tf_pub");
    
    ros::NodeHandle nh("~");
    std::string odom_topic;


    nh.param<std::string>("odom_topic", odom_topic, "/encoder_odom");
    nh.param<int>("scale_factor",gain, 50);

    std::cout << "Listening to Odometry at: " << odom_topic << std::endl;
    std::cout << "Scaling Twist by a factor of " << gain << std::endl;


    ros::Subscriber sub = nh.subscribe(odom_topic, 10, &odomCallback);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom_scaled", 10);

    ros::spin();

    return 0;
}