#ifndef gocator_3100_node_H
#define gocator_3100_node_H

//std
#include <iostream>

//this package
#include "lib/src/gocator3100.h"

//ros dependencies
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h> //PCL-ROS interoperability
#include <pcl_conversions/pcl_conversions.h> //conversions from/to PCL/ROS
#include <std_msgs/Empty.h> //snapshot request
#include <visualization_msgs/Marker.h> //publish bounds of gocator field of view
// #include "gocator_3100/PointCloudAsService.h" //custom "snapshot" service
// #include <sensor_msgs/PointCloud2.h> 

//ROS dynamic configure
#include <gocator_3100/gocator_3100_paramsConfig.h>

//enum run mode
enum RunMode {SNAPSHOT=0,PUBLISHER};

/** \brief Gocator3100 ROS wrapping class
 * 
 * Gocator3100 ROS wrapping class
 * 
 * Two running modes:
 *    \* Snapshot upon request
 *    \* Continuous point cloud publisher (not yet implemented) 
 * 
 **/
class Gocator3100Node
{
    protected:
                        
        //Device object with HW API
        Gocator3100::Device *g3100_camera_;
        
        //ros node handle
        ros::NodeHandle nh_;
        
        //Subscriber. Snapshots requests arrive to this topic as an std_msgs::Empty
        ros::Subscriber snapshot_request_;
        
        //Publisher. Snapshots are published through this topic
        ros::Publisher snapshot_publisher_; 
        
        //Publisher. Line markers bounding the camera field of view
        ros::Publisher fov_publisher_; 
        
        //A pcl point cloud, used to get data and publish it
        pcl::PointCloud<pcl::PointXYZ> cloud_; 

        //Marker message bounding the camera field of view
        visualization_msgs::Marker fov_marker_msg_;
        
        //point cloud server
        //ros::ServiceServer pcl_server_; 
        
        //Indicates if a request has arrived
        bool is_request_;         
        
        //node parameters
        double rate_; //loop rate
        std::string frame_name_; //name of the frame of references with respect cloud are published
        RunMode run_mode_;//run mode: The node acts as a server, or a continuous pcl publisher
        bool fov_viz_; // enable field of view visualization
                
        //camera device parameters
        Gocator3100::DeviceConfigs device_params_;
        
    public:
        //constructor
        Gocator3100Node();
        
        //destructor
        ~Gocator3100Node();
        
        //returns run_mode_
        RunMode runMode() const;
        
        //returns is_request_
        bool isRequest() const;
        
        //sets is_request_ to false
        void resetRequest();
        
        //Call to device snapshot acquisition and publish the point cloud
        void publish();
        
        //publish the filed of view wire frame
        void publish_fov();
        
        //returns rate_ value
        double rate() const; 
        
        //returns fov_viz_
        bool isFovViz() const;
                        
        //Service callback implementing the point cloud snapshot
        //bool pointCloudSnapshotService(gocator_3100::PointCloudAsService::Request  & _request, gocator_3100::PointCloudAsService::Response & _reply);
        
    protected: 
        //snapshot request callback
        void snapshotRequestCallback(const std_msgs::Empty::ConstPtr& _msg);
                
};
#endif
