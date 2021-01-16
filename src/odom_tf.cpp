#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


void odomCallback(const nav_msgs::OdometryConstPtr& msg){
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Time t;

    t =  msg->header.stamp; //if online use ros time now

    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);

    q = q.normalize();
    
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, t, "/odom", "/base_link"));
}


int main(int argc, char **argv){

    ros::init(argc, argv, "tf_pub");
    ros::NodeHandle nh("~");

    std::string odom_topic;


    nh.param<std::string>("odom_topic", odom_topic, "/encoder_odom");

    std::cout << "Listening to Odometry at: " << odom_topic << std::endl;

    ros::Subscriber sub = nh.subscribe(odom_topic, 10, &odomCallback);
    // subscribe to pcl
    // publish new pointcloud topic (regis cloud)
    ros::spin();

    return 0;
}