#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>
#include "tf_conversions/tf_eigen.h"
#include "eigen_conversions/eigen_kdl.h"
#include <pcl/common/transforms.h>

/**
 * @brief create an Affine3d rotation_matrix
 */ 
Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * ry * rx;
}



/** @brief A ROS node used to publish pointclouds and odometry from a directory containing pcd and csv/path files
 *  @author Ahmed Abbas - (github: aaxbas)
 */
class PCDReaderNode{

    protected:
    ros::NodeHandle _n;

    public:
    std::string folder, child_frame, parent_frame, path_topic, csv_path;
    int rate, frame_nr=0;
    bool use_path, use_csv, use_tf, publish_sensor_offset;
    std::string pc_frame, sensor_link;
    nav_msgs::Path path;
    nav_msgs::Odometry odom;
    std::ifstream csv;
    tf::TransformBroadcaster br;
    ros::Publisher odom_pub, cloud_pub;
    Eigen::Affine3d Toffset;
    tf::Transform Toffset_tf;
    double x, y, z, ex, ey, ez;


    PCDReaderNode(ros::NodeHandle nh) : _n(ros::NodeHandle()){
        nh.param<std::string>("folder_path", folder, "/home/abbas/DATA/longshaw_short_path_0/registered_cloud_local_map_frame/");
        nh.param<std::string>("sensor_link", sensor_link, "/laser_link");
        nh.param<std::string>("child_frame_id", child_frame, "/base_link");
        nh.param<std::string>("parent_frame_id", parent_frame, "/odom");
        nh.param<std::string>("path_topic", path_topic, "/lio_sam/mapping/path");
        nh.param<std::string>("csv_path", csv_path, "/home/abbas/DATA/longshaw_short_path_0/trajectory.csv");
        nh.param<std::string>("pc_frame", pc_frame, "/base_link");

        
        
        nh.param<double>("sensor_x",  x, 0);
        nh.param<double>("sensor_y",  y, 0);
        nh.param<double>("sensor_z",  z, 0);
        nh.param<double>("sensor_ex", ex, 0);
        nh.param<double>("sensor_ey", ey, 0);
        nh.param<double>("sensor_ez", ez, 0);
        
        Eigen::Affine3d r = create_rotation_matrix(ex, ey, ez);
        Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(x, y, z)));
        
        Toffset = r*t;
        tf::poseEigenToTF(Toffset, Toffset_tf);



        nh.param<bool>("read_csv", use_csv, true);
        nh.param<bool>("use_path", use_path, false);
        nh.param<bool>("use_tf",use_tf, true);
        nh.param<bool>("publish_sensor_offset",publish_sensor_offset, false);

        nh.param<int>("rate", rate, 10);

        if(use_path){

            ros::Subscriber pathSub = _n.subscribe<nav_msgs::Path>(path_topic, 10, &PCDReaderNode::pathCallback, this);
        
        } else if (use_csv){

            csv.open(csv_path);

            if(csv.eof() || !csv){
                ROS_ERROR("The file at path %s doesn't exist",csv_path.c_str());
                exit(1);
            }
            
            readCSV(csv);

            odom_pub = _n.advertise<nav_msgs::Odometry> ("encoder_odom", 10);
        }
        
        cloud_pub = _n.advertise<sensor_msgs::PointCloud2> ("output_cloud", 10);

        std::cout << "Opening pcd files at directory: " << folder << std::endl;

    }

    void pathCallback(const nav_msgs::Path::ConstPtr &p){
        std::cout << "Message received!" << std::endl;
        path = *p;
    }

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
    /**
     * Read point cloud from pcd file into the cloud_msg param
     * @param count the file number
     * @param cloud_msg the pointcloud msg to read into
     * @return true if the pointcloud was read successfully, false otherwise
     */ 
    bool readPointCloud(int count, sensor_msgs::PointCloud2 &cloud_msg){
        std::string fn = std::string(6 - std::to_string(count).length(), '0') + std::to_string(count);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (folder+fn+".pcd", *cloud) == -1) //* load the file
        {
            ROS_INFO("No more point clouds to read!");
            return (false);
        }
        pcl::transformPointCloud(*cloud, *cloud, Toffset.cast<double>().matrix());
        pcl::toROSMsg(*cloud, cloud_msg);
        return true;
    }

    /**
     * Get odometry from a path message
     * @return true if the path message is received
     */ 
    bool getPath(int count, sensor_msgs::PointCloud2 &cloud_msg){
        
        if(use_path && path.poses.empty()){
            std::cout << "path not received yet at: " << path_topic << std::endl;
            return false;
        } else if (use_path) {
            std::cout << path.poses.size() << std::endl;
        }

        if(use_tf){
            tf::Stamped<tf::Pose> tp;
            tf::Transform t;

            tf::poseStampedMsgToTF(path.poses[count], tp);
            
            t.setOrigin(tp.getOrigin());
            t.setRotation(tp.getRotation());

            ros::Time time(path.poses[count].header.seq);

            br.sendTransform(tf::StampedTransform(t, time, child_frame, child_frame));
            cloud_msg.header.seq = path.poses[count].header.seq;
        }

        return true;
    }

    /**
     * Publish pointcloud and odometry data, transformed if need be
     * @return false if there are no more frames to publish
     */
    bool processFrame(){
        std::cout << "Frame: " << frame_nr << std::endl;

        sensor_msgs::PointCloud2 output;

        if(!readPointCloud(frame_nr, output)){
            return false;
        }

        if (use_path){
            if(!getPath(frame_nr, output)){
                return true;
            };
        } else if (use_csv){
            nav_msgs::Odometry odom = toOdom(readCSV(csv));
            odom.header.frame_id = parent_frame;  // odom frame
            odom.child_frame_id = child_frame;    // base_link
            

            tf::Pose tp;
            tf::poseMsgToTF(odom.pose.pose, tp);
            //tp.getOrigin().setX(-tp.getOrigin().getX());
            //tp.getOrigin().setY(-tp.getOrigin().getY());
            tf::Quaternion q_rot, q = tp.getRotation();
            
            q_rot.setRPY(0, 0, ez);//#3.1415926535); //TODO: Make this not hardcoded
            q = q_rot * q;  //  q  = q_rot * q; 
            tp.setRotation(q);

            /**
             *   tf::Pose tp;
            tf::poseMsgToTF(odom.pose.pose, tp);

            tp.setRotation(tp.getRotation()*Toffset_tf.getRotation());
            
            geometry_msgs::Pose pose;
            tf::poseTFToMsg(tp, pose);
            odom.pose.pose = pose;
            odom_pub.publish(odom);
            */

            geometry_msgs::Pose pose;
            tf::poseTFToMsg(tp, pose);
            odom.pose.pose = pose;
            odom_pub.publish(odom);

            output.header.seq = odom.header.seq;

            if (use_tf){
                tf::Transform t;
                tf::Pose tpo;
                tf::poseMsgToTF(odom.pose.pose, tpo);

                t.setOrigin(tpo.getOrigin());
                t.setRotation(tpo.getRotation());

                ros::Time time(odom.header.seq);

                br.sendTransform(tf::StampedTransform(t,          time, parent_frame, child_frame));
                if(publish_sensor_offset)
                  br.sendTransform(tf::StampedTransform(Toffset_tf, time, child_frame,  sensor_link));
            }
        }

        output.header.frame_id = pc_frame;  // laser_link

        cloud_pub.publish(output);
        frame_nr++;
        return true;
    }
        

};



int main(int argc, char** argv){


    ros::init(argc, argv, "pcd_to_pcl");

    ros::NodeHandle nh("~");

    PCDReaderNode r(nh);
   
    ros::Rate loop_rate(10);
    for(int i = 0; i < 50; i++)
        loop_rate.sleep();

    while(ros::ok() && r.processFrame()){

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}
