#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <string>


/**
 * Read odometry from csv file specified at csv_path
 * @return vector of strings containing odometry data
 */ 
std::vector<std::string> readCSV(std::istream& str){

    std::vector<std::string>   result;
    std::string                line;
    std::getline(str,line);

    std::stringstream          lineStream(line);
    std::string                cell;

    while(std::getline(lineStream,cell, ',')) {
        result.push_back(cell);
    }

    // This checks for a trailing comma with no data after it.
    if (!lineStream && cell.empty()) {
        // If there was a trailing comma then add an empty element.
        result.push_back("");
    }

    return result;

}

/**
 * Convert csv odometry data to odometry message
 * @return ROS odometry msg
 */ 
nav_msgs::Odometry toOdom(std::vector<std::string> str_odom){
    nav_msgs::Odometry odom;
    
    ROS_INFO("%s",str_odom[0].c_str());
    
    odom.header.seq = std::stoul(str_odom[0]);
    odom.pose.pose.position.x = std::stod(str_odom[1]);
    odom.pose.pose.position.y = std::stod(str_odom[2]);
    odom.pose.pose.position.z = std::stod(str_odom[3]);
    odom.pose.pose.orientation.x = std::stod(str_odom[4]);
    odom.pose.pose.orientation.y = std::stod(str_odom[5]);
    odom.pose.pose.orientation.z = std::stod(str_odom[6]);
    odom.pose.pose.orientation.w = std::stod(str_odom[7]);

    return odom;
}



int main(int argc, char** argv){
    
    ros::init(argc,argv, "odom_reader");
    ros::NodeHandle n("~"); 

    std::ifstream csv;
    std::string csv_path;

    n.param<std::string>("csv_path", csv_path, "/home/abbas/DATA/courtyard_localization_0/trajectory.csv");
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 10);
    
    ROS_INFO("PATH: %s", csv_path.c_str());

    csv.open(csv_path);

    ros::Rate loop_rate(10);

    readCSV(csv);

    while (ros::ok() && !csv.eof()){
        nav_msgs::Odometry odom = toOdom(readCSV(csv));
        odom.header.frame_id = "odom";

        odom_pub.publish(odom);

        loop_rate.sleep();
    }

}

